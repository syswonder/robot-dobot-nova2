#!/usr/bin/env python3
"""Executor calls and a read-only runtime check while rbnx boot is running."""
from __future__ import annotations

import json
from pathlib import Path
import sys
import uuid

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "primitives/nova2_arm/rbnx-build/codegen/proto_gen"))

import grpc
import pilot_pb2
import robonix_contracts_pb2_grpc


def call_tools(calls):
    run_id = "nova2-cli-" + uuid.uuid4().hex
    nodes = [pilot_pb2.RtdlNode(node_kind=0, children=list(range(1, len(calls) + 1)), op_id="root", description="Nova2 tool calls")]
    for name, arguments in calls:
        nodes.append(pilot_pb2.RtdlNode(node_kind=2, op_id=name, description="Nova2 " + name,
            call=pilot_pb2.CapabilityCall(call_id=name, provider_id="nova2_arm",
                contract_id="robonix/primitive/dobot_nova2/" + name, args_json=json.dumps(arguments))))
    plan = pilot_pb2.Plan(plan_id=run_id, session_id=run_id, round=1, root_index=0, nodes=nodes)
    results = {}
    with grpc.insecure_channel("127.0.0.1:50061") as channel:
        for event in robonix_contracts_pb2_grpc.RobonixSystemExecutorExecuteStub(channel).Execute(plan, timeout=20):
            if event.event_kind != 1 or not event.node_state.leaf_result.call_id:
                continue
            result = event.node_state.leaf_result
            if not result.success:
                raise RuntimeError(result.error or result.output)
            response = json.loads(result.output)
            if not response["ok"]:
                raise RuntimeError(response["message"])
            payload = json.loads(response["result_json"])
            results[result.call_id] = payload
    if set(results) != {name for name, _ in calls}:
        raise RuntimeError(f"Missing tool results: {set(results)}")
    return results


def main():
    results = call_tools([("get_state", {}), ("forward_kinematics", {"joints_deg": [0, -20, 40, 0, 40, 0]})])
    if results["get_state"]["dashboard_errors"]:
        raise RuntimeError(results["get_state"]["dashboard_errors"])
    for name, payload in results.items():
        print(name + ":", json.dumps(payload, indent=2, ensure_ascii=False))
    print("PASS: Executor -> MCP -> real controller read-only checks")


if __name__ == "__main__":
    main()
