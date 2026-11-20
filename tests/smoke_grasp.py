"""Real Robonix lifecycle/Executor/MCP/VLM pipeline, local fake hardware only."""
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import shlex
import shutil
import socket
import subprocess
import sys
import tempfile
import threading
import time
import uuid

import cv2
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]
for directory in ('tests','scripts','primitives/nova2_arm','primitives/tg9801_gripper/tests',
                  'primitives/orbbec_camera','primitives/orbbec_camera/tests',
                  'skills/nova2_grasp/rbnx-build/codegen/proto_gen'):
    sys.path.insert(0,str(ROOT/directory))
from fake_robot import FakeRobot,V3Type
from fake_gripper import FakeGripper
from check_grasp import call_grasp
from orbbec_usb.controller import camera_key
from fake_worker import metadata
import grpc
import pilot_pb2
import robonix_contracts_pb2_grpc


def port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1',0));return sock.getsockname()[1]


class ModelCamera(BaseHTTPRequestHandler):
    vision_calls=0
    agent_calls=0
    agent_cap='nova2_grasp.nova2_grasp_detect_object'
    agent_args={'object':'green block','backend':'vlm'}
    jpeg=cv2.imencode('.jpg',np.zeros((600,800,3),np.uint8))[1].tobytes()

    def log_message(self,*args):pass

    def do_GET(self):
        if self.path.startswith('/api/'):
            self.send_response(500);self.end_headers()  # No HTTP camera fallback may be used.
        else:
            self.send_response(200);self.end_headers();self.wfile.write(b'{"data":[{"id":"local-test-model"}]}')

    def do_POST(self):
        request=json.loads(self.rfile.read(int(self.headers['Content-Length'])))
        vision=any(isinstance(m.get('content'),list) and any(p.get('type')=='image_url' for p in m['content']) for m in request['messages'])
        if vision:
            type(self).vision_calls+=1
            content=json.dumps(dict(found=True,label='green block',confidence=.95,
                bbox_xyxy=[200,200,400,300],top_corners_px=[[200,200],[400,200],[400,300],[200,300]]))
        else:
            type(self).agent_calls+=1;done=type(self).agent_calls>1
            content=json.dumps({'content':'MOCK_GRASP_CHAT_DONE' if done else 'Use the requested Nova2 grasp skill.',
                'rtdl_description':'Nova2 grasp smoke',
                'rtdl':{'op':'sequence','op_id':2,'description':'done','children':[]} if done else
                    {'op':'do','op_id':1,'description':'Call Nova2 grasp skill','cap':self.agent_cap,'args':self.agent_args},
                'task_update':{'goal':'Test grasp skill','success_criterion':'Skill tool accepted',
                    'status':'done' if done else 'in_progress'}})
        self.send_response(200)
        if request.get('stream'):
            self.send_header('Content-Type','text/event-stream');self.end_headers()
            for delta,finish in [({'role':'assistant','content':content},None),({},'stop')]:
                chunk={'id':'local','object':'chat.completion.chunk','model':'local-test-model',
                    'choices':[{'index':0,'delta':delta,'finish_reason':finish}]}
                self.wfile.write(('data: '+json.dumps(chunk)+'\n\n').encode())
            self.wfile.write(b'data: [DONE]\n\n')
        else:
            self.send_header('Content-Type','application/json');self.end_headers()
            self.wfile.write(json.dumps({'choices':[{'message':{'content':content}}]}).encode())


def arm_call(name,args,executor):
    ident=uuid.uuid4().hex
    leaf=pilot_pb2.RtdlNode(node_kind=2,op_id=name,description='Fake arm '+name,call=pilot_pb2.CapabilityCall(call_id=ident,
        provider_id='nova2_arm',contract_id='robonix/primitive/dobot_nova2/'+name,args_json=json.dumps(args)))
    plan=pilot_pb2.Plan(plan_id=ident,session_id=ident,round=1,root_index=0,nodes=[leaf])
    with grpc.insecure_channel(executor) as channel:
        for e in robonix_contracts_pb2_grpc.RobonixSystemExecutorExecuteStub(channel).Execute(plan,timeout=15):
            r=e.node_state.leaf_result
            if e.event_kind==1 and r.call_id==ident:
                assert r.success,(r.error,r.output)
                response=json.loads(r.output);assert response['ok'],response
                return json.loads(response['result_json'])
    raise RuntimeError('No arm result')


def main():
    artifacts = ROOT / '.artifacts'
    artifacts.mkdir(exist_ok=True)
    folder = Path(tempfile.mkdtemp(prefix='grasp-smoke-', dir=artifacts))
    robot=FakeRobot('v3');gripper=FakeGripper(adaptive_holding_status=3)
    gripper.config['adaptive_status_scheme']='three_is_holding'
    model=ThreadingHTTPServer(('127.0.0.1',0),ModelCamera)
    threading.Thread(target=model.serve_forever,daemon=True).start()
    ready={'user':0,'tool':0,'joints_deg':robot.joints.copy(),'pose_mm_deg':robot.pose.copy()}
    key=camera_key(metadata({'color_width':800,'color_height':600}))
    cal={'format':'nova2-planar-handeye-v1','accepted':True,'frames':{'user':0,'tool':0},
        'camera_key':key,'image_size':[800,600],'pixel_to_xy':[[.1,0,140],[0,.1,-300],[0,0,1]],
        'valid_pixel_polygon':[[0,0],[799,0],[799,599],[0,599]],'samples':9,
        'max_error_mm':.5,'z_mm':100,'orientation_rxryrz_deg':[180,0,90]}
    places={'白盒子':{'xy_mm':[190,-260],'release_height_mm':150}}
    for name,value in [('calibration.json',cal),('ready.json',ready),('place.json',places)]:
        (folder/name).write_text(json.dumps(value))
    manifest=yaml.safe_load((ROOT/'robonix_manifest.yaml').read_text())
    for name in manifest['system']:
        manifest['system'][name]['listen']='127.0.0.1:'+str(port())
    manifest['system']['liaison']['pilot_endpoint']=manifest['system']['pilot']['listen']
    manifest['system']['soma']['robot_yaml']=str(ROOT/'soma.yaml')
    endpoint=f'http://127.0.0.1:{model.server_port}'
    manifest['system']['pilot']['vlm']={'upstream':endpoint+'/v1','api_key':'local-only','model':'local-test-model','api_format':'openai'}
    manifest['primitive'][0].update(path=str(ROOT/'primitives/nova2_arm'),config={**robot.config,'feedback_timeout_s':1.5})
    manifest['primitive'][1].update(path=str(ROOT/'primitives/tg9801_gripper'),config=gripper.config)
    # Production MCP + lifecycle + frame parser, only USB SDK replaced by a pipe fixture.
    camera_pkg=folder/'camera';camera_pkg.mkdir()
    camera_source=ROOT/'primitives/orbbec_camera'
    shutil.copytree(camera_source/'capabilities',camera_pkg/'capabilities')
    shutil.copyfile(camera_source/'config.spec',camera_pkg/'config.spec')
    camera_manifest=yaml.safe_load((camera_source/'package_manifest.yaml').read_text())
    camera_manifest['build']='true';camera_manifest['start']='bash start.sh'
    (camera_pkg/'package_manifest.yaml').write_text(yaml.safe_dump(camera_manifest,sort_keys=False))
    paths=[str(camera_source),str(camera_source/'rbnx-build/codegen/proto_gen'),str(camera_source/'rbnx-build/codegen/robonix_mcp_types')]
    (camera_pkg/'start.sh').write_text('export PYTHONPATH='+shlex.quote(':'.join(paths))+'\nexec '+
        shlex.quote(str(ROOT/'skills/nova2_grasp/rbnx-build/venv/bin/python'))+' '+shlex.quote(str(ROOT/'tests/fake_camera_provider.py'))+'\n')
    manifest['primitive'][2].update(path=str(camera_pkg),config=dict(color_width=800,color_height=600,
        serial_number='fixture',expected_camera_key=key,startup_timeout_s=5.))
    manifest['skill'][0].update(path=str(ROOT/'skills/nova2_grasp'),config={
        'calibration_path':str(folder/'calibration.json'),'ready_pose_path':str(folder/'ready.json'),
        'place_locations_path':str(folder/'place.json'),
        'runtime_path':str(folder/'runtime.json'),'artifacts_dir':str(folder/'results'),
        'camera_provider_id':'orbbec_camera',
        'vlm_base_url':endpoint+'/v1','vlm_api_key':'local-only','vlm_model':'local-test-model',
        'jaw_alignment_confirmed':True,'approach_height_mm':200.,'velocity_percent':1})
    file=folder/'robonix_manifest.yaml';file.write_text(yaml.safe_dump(manifest,sort_keys=False))
    boot=None
    try:
        log=folder/'boot.log'
        with log.open('wb') as output:
            # rbnx writes boot state relative to cwd even with -f. Never share
            # the live body's state directory when testing an isolated stack.
            boot=subprocess.Popen(['rbnx','boot','-f',str(file),'--no-update-check','-v'],cwd=folder,stdout=output,stderr=output)
        deadline=time.monotonic()+60
        while time.monotonic()<deadline:
            if boot.poll() is not None:raise RuntimeError('Fake stack exited: '+str(log))
            if 'all components up' in log.read_text(errors='replace'):break
            time.sleep(.2)
        else:raise RuntimeError('Fake stack timeout: '+str(log))
        executor=manifest['system']['executor']['listen']
        status=call_grasp('status',executor=executor);assert status['task']['state']=='IDLE'
        assert status['place_locations']['白盒子']['release_z_mm']==250
        assert not gripper.writes
        plan=call_grasp('plan_grasp',{'object':'green block','backend':'vlm'},executor)
        assert plan['ik_checked'] and plan['hover_pose_mm_deg'][2]==300
        assert not gripper.writes
        assert not any(c.startswith(('EnableRobot','JointMovJ','MovJ','MovL')) for c in robot.commands)
        # Disabled robot must fail without automatically enabling or opening.
        call_grasp('pick',{'object':'','backend':'','plan_id':plan['plan_id']},executor)
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            state=call_grasp('task_status',executor=executor)
            if state['state']!='RUNNING':break
            time.sleep(.1)
        assert state['state']=='FAILED' and not gripper.writes,state
        arm_call('enable',{},executor)
        call_grasp('pick',{'object':'green block','backend':'vlm','plan_id':''},executor)
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            state=call_grasp('task_status',executor=executor)
            if state['state']!='RUNNING':break
            time.sleep(.1)
        (folder/'result.json').write_text(json.dumps(state,ensure_ascii=False,indent=2))
        assert state['state']=='SUCCEEDED',state
        assert state['result']['at_hover'] and state['result']['gripper']['holding']
        assert np.allclose(robot.pose,state['result']['plan']['hover_pose_mm_deg'])
        assert any(addr==10 and value==[300,0] for addr,value in gripper.writes)
        assert any(addr==10 and value==[300,1] for addr,value in gripper.writes)
        # Release, inspect hover, then exercise the new operator-approved
        # descend/grip/lift/replace/release/retract/home tool over real MCP.
        for action,args in [('release',{}),('approach',{'object':'green block','backend':'vlm','plan_id':''}),('cycle_from_hover',{})]:
            call_grasp(action,args,executor)
            deadline=time.monotonic()+30
            while time.monotonic()<deadline:
                state=call_grasp('task_status',executor=executor)
                if state['state']!='RUNNING':break
                time.sleep(.1)
            assert state['state']=='SUCCEEDED',(action,state)
        assert state['result']['object_placed_back'] and not state['result']['release_feedback']['holding']
        call_grasp('pick_and_place',{'object':'green block','backend':'vlm','plan_id':'','destination':'白盒子'},executor)
        deadline=time.monotonic()+30
        while time.monotonic()<deadline:
            state=call_grasp('task_status',executor=executor)
            if state['state']!='RUNNING':break
            time.sleep(.1)
        assert state['state']=='SUCCEEDED',state
        assert state['result']['destination']=='白盒子' and state['result']['released']
        assert np.allclose(robot.pose[:3],[190,-260,250])
        assert not state['result']['gripper']['holding']
        # Real Liaison -> Pilot -> Executor -> skill path, scripted model.
        task=pilot_pb2.Task(task_id=uuid.uuid4().hex,session_id=uuid.uuid4().hex,source=0,
            text='Use Nova2 grasp skill to detect the green block only; never move or release.',
            context_json='{"user_id":"grasp-smoke","modality":"text"}',timestamp_ms=int(time.time()*1000))
        motion_count=sum(c.startswith(('JointMovJ','MovJ','MovL')) for c in robot.commands)
        writes=len(gripper.writes);seen=False
        with grpc.insecure_channel(manifest['system']['liaison']['listen']) as channel:
            for event in robonix_contracts_pb2_grpc.RobonixSystemLiaisonSubmitStub(channel).SubmitTask(task,timeout=35):
                leaf=event.node_state.leaf_result
                if leaf.contract_id=='robonix/skill/nova2_grasp/detect_object':
                    assert leaf.success and json.loads(leaf.output)['ok'],leaf
                    seen=True
        assert seen and len(gripper.writes)==writes
        assert sum(c.startswith(('JointMovJ','MovJ','MovL')) for c in robot.commands)==motion_count
        assert status['camera']['connected']
        print('PASS: SDK pipe camera -> MCP -> skill, VLM/IK, hover holding, named placement, supervised cycle, Liaison chat')
        print('Artifacts:',folder)
    finally:
        if boot and boot.poll() is None:
            boot.send_signal(signal.SIGINT)
            try:boot.wait(timeout=15)
            except subprocess.TimeoutExpired:boot.kill();boot.wait()
        model.shutdown();model.server_close();robot.close();gripper.close()


if __name__=='__main__':main()
