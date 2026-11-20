#!/usr/bin/env python3
"""Call Nova2 grasp through Executor. Default status is read-only."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'skills/nova2_grasp/rbnx-build/codegen/proto_gen'))
import grpc
import pilot_pb2
import robonix_contracts_pb2_grpc


def call_grasp(name, args=None, executor="127.0.0.1:50061", timeout=90):
    ident = "nova2-grasp-"+uuid.uuid4().hex
    leaf = pilot_pb2.RtdlNode(node_kind=2, op_id=name, description="Nova2 grasp "+name,
        call=pilot_pb2.CapabilityCall(call_id=ident, provider_id="nova2_grasp",
            contract_id="robonix/skill/nova2_grasp/"+name, args_json=json.dumps(args or {})))
    plan = pilot_pb2.Plan(plan_id=ident, session_id=ident, round=1, root_index=0, nodes=[leaf])
    with grpc.insecure_channel(executor) as channel:
        for event in robonix_contracts_pb2_grpc.RobonixSystemExecutorExecuteStub(channel).Execute(plan, timeout=timeout):
            result = event.node_state.leaf_result
            if event.event_kind != 1 or result.call_id != ident:
                continue
            if not result.success:
                raise RuntimeError(result.error or result.output)
            response = json.loads(result.output)
            if not response.get("ok"):
                raise RuntimeError(response.get("message", "Skill call failed"))
            return json.loads(response["result_json"])
    raise RuntimeError("Executor returned no skill result")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executor', default='127.0.0.1:50061')
    parser.add_argument('--action', default='status', choices=('status','detect_object','plan_grasp','approach','pick','pick_and_place','cycle_from_hover','home','release','stop','task_status','set_jaw_alignment'))
    parser.add_argument('--args', default='{}', help='JSON tool arguments; explicit actions may move hardware')
    parser.add_argument('--wait', action='store_true', help='Poll an accepted asynchronous task; no automatic retry')
    args = parser.parse_args()
    result = call_grasp(args.action, json.loads(args.args), args.executor)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    if args.wait and result.get('state') == 'RUNNING':
        run_id = result['run_id']
        while result['state'] == 'RUNNING':
            time.sleep(.5)
            result = call_grasp('task_status', executor=args.executor)
            if result['run_id'] != run_id:
                raise RuntimeError('Latest task changed while waiting')
            print(result['state'], result.get('stage'), result['message'], flush=True)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        if result['state'] != 'SUCCEEDED':
            raise SystemExit(1)


if __name__=='__main__':main()
