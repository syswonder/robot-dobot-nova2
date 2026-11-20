#!/usr/bin/env python3
"""Read-only Orbbec MCP camera diagnostic; no calibration HTTP or robot writes."""
import argparse
import base64
import json
from pathlib import Path
import sys
import uuid

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'primitives/orbbec_camera/rbnx-build/codegen/proto_gen'))
import grpc
import pilot_pb2
import robonix_contracts_pb2_grpc


def call_camera(name,args=None,executor='127.0.0.1:50061',timeout=45):
    ident='camera-'+uuid.uuid4().hex
    node=pilot_pb2.RtdlNode(node_kind=2,op_id=name,description='Read Orbbec camera '+name,
        call=pilot_pb2.CapabilityCall(call_id=ident,provider_id='orbbec_camera',
            contract_id='robonix/primitive/orbbec_camera/'+name,args_json=json.dumps(args or {})))
    plan=pilot_pb2.Plan(plan_id=ident,session_id=ident,round=1,root_index=0,nodes=[node])
    with grpc.insecure_channel(executor) as channel:
        for event in robonix_contracts_pb2_grpc.RobonixSystemExecutorExecuteStub(channel).Execute(plan,timeout=timeout):
            result=event.node_state.leaf_result
            if event.event_kind!=1 or result.call_id!=ident:continue
            if not result.success:raise RuntimeError(result.error or result.output)
            response=json.loads(result.output)
            if not response.get('ok'):raise RuntimeError(response.get('message','Camera tool failed'))
            return json.loads(response['result_json'])
    raise RuntimeError('No camera result')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--executor',default='127.0.0.1:50061')
    p.add_argument('--save',type=Path,help='Optional JPEG output path; no snapshot requested otherwise')
    a=p.parse_args()
    print(json.dumps(call_camera('get_state',executor=a.executor),ensure_ascii=False,indent=2))
    if a.save:
        frame=call_camera('snapshot',executor=a.executor)
        a.save.parent.mkdir(parents=True,exist_ok=True)
        a.save.write_bytes(base64.b64decode(frame.pop('jpeg_base64'),validate=True))
        print('JPEG saved:',a.save)
        print(json.dumps(frame,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
