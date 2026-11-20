"""Exercise the real chat path against a local robot and scripted model.

No hardware or external model credentials are used.
"""
from __future__ import annotations

from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
import uuid

import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "primitives/nova2_arm"))
sys.path.insert(0, str(ROOT / "primitives/nova2_arm/rbnx-build/codegen/proto_gen"))
sys.path.insert(0, str(ROOT / "primitives/tg9801_gripper/tests"))
sys.path.insert(0, str(ROOT / "scripts"))
import grpc
import pilot_pb2
import robonix_contracts_pb2_grpc
from fake_robot import FakeRobot
from fake_gripper import FakeGripper
from check_gripper import call_gripper


def free_port():
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


class Model(BaseHTTPRequestHandler):
    calls = 0
    request_path = None
    capability = "nova2_arm.dobot_nova2_get_state"
    arguments = {}

    def log_message(self, *_args):
        pass

    def do_GET(self):
        data = json.dumps({"object": "list", "data": [{"id": "local-test-model", "object": "model"}]}).encode()
        self.send_response(200); self.send_header("Content-Type", "application/json")
        self.end_headers(); self.wfile.write(data)

    def do_POST(self):
        request = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        if Model.request_path is not None:
            Model.request_path.write_text(json.dumps(request, indent=2, ensure_ascii=False))
        Model.calls += 1
        done = Model.calls > 1
        envelope = {
            "content": "MOCK_CHAT_STATE_OK" if done else "Read the local Nova2 test controller state.",
            "rtdl_description": "Read-only smoke test",
            "rtdl": {"op": "sequence", "op_id": 2, "description": "Complete", "children": []} if done else {
                "op": "do", "op_id": 1, "description": "Read test controller state",
                "cap": Model.capability, "args": Model.arguments},
            "task_update": {"goal": "Read test controller state", "success_criterion": "State tool returned measured data",
                            "status": "done" if done else "in_progress"},
        }
        content = json.dumps(envelope)
        self.send_response(200)
        if request.get("stream"):
            self.send_header("Content-Type", "text/event-stream"); self.end_headers()
            for delta, finish in [({"role": "assistant", "content": content}, None), ({}, "stop")]:
                chunk = {"id": "chatcmpl-local", "object": "chat.completion.chunk", "created": int(time.time()),
                         "model": "local-test-model", "choices": [{"index": 0, "delta": delta, "finish_reason": finish}]}
                self.wfile.write(("data: " + json.dumps(chunk) + "\n\n").encode())
            self.wfile.write(b"data: [DONE]\n\n")
        else:
            self.send_header("Content-Type", "application/json"); self.end_headers()
            self.wfile.write(json.dumps({"id": "chatcmpl-local", "object": "chat.completion",
                "created": int(time.time()), "model": "local-test-model", "choices": [{"index": 0,
                "message": {"role": "assistant", "content": content}, "finish_reason": "stop"}]}).encode())


def main():
    robot = FakeRobot("v3")
    gripper = FakeGripper()
    model = ThreadingHTTPServer(("127.0.0.1", 0), Model)
    threading.Thread(target=model.serve_forever, daemon=True).start()
    artifacts = ROOT / ".artifacts"
    artifacts.mkdir(exist_ok=True)
    folder = Path(tempfile.mkdtemp(prefix="chat-smoke-", dir=artifacts))
    Model.request_path = folder / "model-request.json"
    manifest = yaml.safe_load((ROOT / "robonix_manifest.yaml").read_text())
    endpoint = "127.0.0.1:" + str(free_port())
    manifest["system"]["atlas"]["listen"] = endpoint
    for name in ("executor", "soma", "pilot", "liaison"):
        manifest["system"][name]["listen"] = "127.0.0.1:" + str(free_port())
    manifest["system"]["liaison"]["pilot_endpoint"] = manifest["system"]["pilot"]["listen"]
    manifest["system"]["soma"]["robot_yaml"] = str(ROOT / "soma.yaml")
    manifest["primitive"][0]["path"] = str(ROOT / "primitives/nova2_arm")
    manifest["primitive"][0]["config"] = robot.config
    manifest["primitive"][1]["path"] = str(ROOT / "primitives/tg9801_gripper")
    manifest["primitive"][1]["config"] = gripper.config
    manifest["system"]["pilot"]["vlm"] = {"upstream": f"http://127.0.0.1:{model.server_port}/v1",
                                               "api_key": "local-test-only", "model": "local-test-model", "api_format": "openai"}
    filename = folder / "robonix_manifest.yaml"
    filename.write_text(yaml.safe_dump(manifest, sort_keys=False))
    boot = None
    log_path = folder / "boot.log"
    try:
        with log_path.open("w") as log:
            boot = subprocess.Popen(["rbnx", "boot", "-f", str(filename), "--no-update-check", "-v"],
                                    cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 25
            while time.monotonic() < deadline:
                if boot.poll() is not None:
                    raise RuntimeError("Smoke-test boot exited; inspect " + str(log_path))
                if "all components up" in log_path.read_text():
                    break
                time.sleep(.2)
            else:
                raise RuntimeError("Smoke-test startup timed out; inspect " + str(log_path))
            executor = manifest["system"]["executor"]["listen"]
            info = call_gripper("info", executor=executor)
            if info["model"] != "HKV TG-9801" or gripper.writes:
                raise RuntimeError("TG9801 startup/info not read-only")
            call_gripper("grip", {"speed": 250, "force_percent": 10}, executor=executor)
            state = call_gripper("wait_for_state", {"condition": "holding", "timeout_s": 1}, executor=executor)
            if not state["state"]["holding"]:
                raise RuntimeError("Gripper measured holding feedback failed")
            print("PASS: Executor -> TG9801 MCP -> serial -> Modbus RTU emulator")
            result = subprocess.run(["rbnx", "ask", "--server", endpoint, "Read Nova2 state only; do not enable or move."],
                                    cwd=ROOT, capture_output=True, text=True, timeout=30)
            (folder / "ask.log").write_text(result.stdout + result.stderr)
            print(result.stdout)
            if result.returncode or "[✓ get_state]" not in result.stdout or Model.calls < 2:
                raise RuntimeError("Ask path failed; inspect " + str(folder / "ask.log"))
            print("PASS: rbnx ask -> Pilot -> Executor -> MCP -> local V3 controller")

            Model.calls = 0
            command_count = len(robot.commands)
            task = pilot_pb2.Task(task_id=uuid.uuid4().hex, session_id=uuid.uuid4().hex, source=0,
                text="Read Nova2 state only; do not enable or move.",
                context_json=json.dumps({"user_id": "nova2-smoke-test", "modality": "text"}),
                timestamp_ms=int(time.time() * 1000))
            received_state = False
            with grpc.insecure_channel(manifest["system"]["liaison"]["listen"]) as channel:
                client = robonix_contracts_pb2_grpc.RobonixSystemLiaisonSubmitStub(channel)
                for event in client.SubmitTask(task, timeout=30):
                    leaf = event.node_state.leaf_result
                    if leaf.contract_id == "robonix/primitive/dobot_nova2/get_state":
                        response = json.loads(leaf.output) if leaf.output else {}
                        if not leaf.success or not response.get("ok"):
                            raise RuntimeError("Liaison state call failed: " + (leaf.error or leaf.output))
                        if json.loads(response["result_json"])["protocol"] != "v3":
                            raise RuntimeError("Unexpected robot state through Liaison")
                        received_state = True
            if not received_state or not any(c == "GetAngle()" for c in robot.commands[command_count:]):
                raise RuntimeError("Liaison chat entry did not query the test robot")
            if not any(c == "GetAngle()" for c in robot.commands):
                raise RuntimeError("Chat did not query the test robot")
            if any(c.startswith(("EnableRobot", "JointMovJ", "MovJ", "MovL")) for c in robot.commands):
                raise RuntimeError("Read-only chat issued an unexpected write")
            print("PASS: chat SubmitTask -> Liaison -> Pilot -> Executor -> MCP -> local V3 controller")
            Model.calls = 0
            Model.capability = "tg9801_gripper.tg9801_open"
            Model.arguments = {"speed": 300}
            task.task_id = uuid.uuid4().hex
            task.session_id = uuid.uuid4().hex
            task.text = "Open only the TG9801 gripper at speed 300; never move the arm."
            received_gripper = False
            before = len(gripper.writes)
            with grpc.insecure_channel(manifest["system"]["liaison"]["listen"]) as channel:
                client = robonix_contracts_pb2_grpc.RobonixSystemLiaisonSubmitStub(channel)
                for event in client.SubmitTask(task, timeout=30):
                    leaf = event.node_state.leaf_result
                    if leaf.contract_id == "robonix/primitive/tg9801/open":
                        if not leaf.success or not json.loads(leaf.output).get("ok"):
                            raise RuntimeError("Chat gripper call failed: " + leaf.output)
                        received_gripper = True
            if not received_gripper or len(gripper.writes) != before + 2 or gripper.writes[-1] != (10, [300, 0]):
                raise RuntimeError("Chat gripper command did not reach Modbus emulator")
            if any(c.startswith(("EnableRobot", "JointMovJ", "MovJ", "MovL")) for c in robot.commands):
                raise RuntimeError("Gripper chat command moved the arm")
            print("PASS: TG9801 chat SubmitTask -> Liaison -> Pilot -> Executor -> MCP -> Modbus")
            print("Model replies were scripted; this checks integration, not model reasoning.")
    finally:
        if boot and boot.poll() is None:
            boot.send_signal(signal.SIGINT)
            try:
                boot.wait(timeout=12)
            except subprocess.TimeoutExpired:
                boot.kill(); boot.wait()
        model.shutdown(); model.server_close(); robot.close(); gripper.close()


if __name__ == "__main__":
    main()
