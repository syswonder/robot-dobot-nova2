#!/usr/bin/env python3
"""TG9801 through Executor/MCP. Default is read-only; actions are explicit."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import uuid

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "primitives/tg9801_gripper/rbnx-build/codegen/proto_gen"))
import grpc
import pilot_pb2
import robonix_contracts_pb2_grpc


def call_gripper(name, args=None, executor="127.0.0.1:50061", timeout=20):
    ident = "gripper-" + uuid.uuid4().hex
    leaf = pilot_pb2.RtdlNode(node_kind=2, op_id=name, description="TG9801 " + name,
        call=pilot_pb2.CapabilityCall(call_id=name, provider_id="tg9801_gripper",
            contract_id="robonix/primitive/tg9801/" + name, args_json=json.dumps(args or {})))
    plan = pilot_pb2.Plan(plan_id=ident, session_id=ident, round=1, root_index=0, nodes=[leaf])
    with grpc.insecure_channel(executor) as channel:
        for event in robonix_contracts_pb2_grpc.RobonixSystemExecutorExecuteStub(channel).Execute(plan, timeout=timeout):
            result = event.node_state.leaf_result
            if event.event_kind != 1 or result.call_id != name:
                continue
            if not result.success:
                raise RuntimeError(result.error or result.output)
            response = json.loads(result.output)
            if not response.get("ok"):
                raise RuntimeError(response.get("message", str(response)))
            return json.loads(response["result_json"])
    raise RuntimeError("Executor returned no gripper tool result")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--executor", default="127.0.0.1:50061")
    parser.add_argument("--action", choices=("info", "get_state", "open", "close", "grip", "set_parameters", "wait_for_state", "move_position"))
    parser.add_argument("--args", default="{}", help="JSON tool arguments")
    args = parser.parse_args()
    for name in ([args.action] if args.action else ["info", "get_state"]):
        result = call_gripper(name, json.loads(args.args), executor=args.executor)
        print(name, json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
