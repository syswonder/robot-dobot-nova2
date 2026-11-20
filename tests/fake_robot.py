"""Local TCP controller used only by tests; never connects to real hardware."""
from __future__ import annotations

import re
import socketserver
import threading
import time

import numpy as np

from nova2_arm.vendor.dobot_api import MyType
from nova2_arm.vendor.dobot_api_v3 import MyType as V3Type


class Server(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True
    block_on_close = False


class FakeRobot:
    def __init__(self, protocol="v3"):
        self.protocol = protocol
        self.mode = 4
        self.joints = [0., -20., 40., 0., 40., 0.]
        self.pose = [100., -200., 300., 180., 0., 90.]
        self.command_id = 0
        self.pending = None
        self.duration = .35
        self.commands = []
        self.reject_next_motion = False
        self.drop_next_motion_ack = False
        self.bad_feedback = False
        self.halt_feedback = False
        self.wrong_target = False
        self.error = False
        self.ik_pose = None
        self.ik_joints = None
        self.closed = threading.Event()
        self.lock = threading.RLock()
        owner = self

        class Commands(socketserver.BaseRequestHandler):
            def handle(self):
                data = ""
                while not owner.closed.is_set():
                    part = self.request.recv(4096)
                    if not part:
                        return
                    data += part.decode()
                    while ")" in data:
                        command, data = data.split(")", 1)
                        command += ")"
                        with owner.lock:
                            code, payload, drop = owner.command(command)
                        if drop:
                            return
                        reply = f"{code},{{{payload}}},{command};".encode()
                        # Exercise reply framing across multiple TCP reads.
                        self.request.sendall(reply[:7])
                        self.request.sendall(reply[7:])

        class Stream(socketserver.BaseRequestHandler):
            def handle(self):
                try:
                    while not owner.closed.wait(.02):
                        with owner.lock:
                            if owner.halt_feedback:
                                continue
                            packet = owner.packet()
                        self.request.sendall(packet[:43])
                        self.request.sendall(packet[43:])
                except OSError:
                    return

        self.dashboard = Server(("127.0.0.1", 0), Commands)
        self.motion = Server(("127.0.0.1", 0), Commands)
        self.feedback = Server(("127.0.0.1", 0), Stream)
        self.servers = [self.dashboard, self.motion, self.feedback]
        for s in self.servers:
            threading.Thread(target=s.serve_forever, kwargs={"poll_interval": .02}, daemon=True).start()

    @property
    def config(self):
        return {"robot_ip": "127.0.0.1", "protocol": self.protocol,
                "dashboard_port": self.dashboard.server_address[1],
                "motion_port": self.motion.server_address[1],
                "feedback_port": self.feedback.server_address[1],
                "socket_timeout_s": .5, "feedback_timeout_s": .3,
                "motion_timeout_s": 3.0}

    def command(self, command):
        self.commands.append(command)
        name = command.split("(", 1)[0]
        if name == "RobotMode":
            return 0, str(self.mode), False
        if name == "GetAngle":
            return 0, ",".join(map(str, self.joints)), False
        if name == "GetPose":
            if self.protocol == "v3" and command != "GetPose(User=0,Tool=0)":
                return -40001, "", False
            return 0, ",".join(map(str, self.pose)), False
        if name == "GetErrorID":
            return 0, "[[],[],[],[],[],[],[]]", False
        if name == "EnableRobot":
            self.mode = 5
        elif name == "DisableRobot":
            self.mode = 4
        elif name in ("ResetRobot", "Stop"):
            self.pending = None
            self.mode = 5 if self.mode != 4 else 4
        elif name in ("PositiveSolution", "PositiveKin"):
            return 0, ",".join(map(str, self.pose)), False
        elif name in ("InverseSolution", "InverseKin"):
            self.ik_pose = [float(v) for v in re.findall(r"-?\d+(?:\.\d+)?", command.split("(", 1)[1])[:6]]
            self.ik_joints = self.joints.copy()
            self.ik_joints[0] += 5
            return 0, ",".join(map(str, self.ik_joints)), False
        elif name in ("JointMovJ", "MovJ", "MovL"):
            if self.reject_next_motion:
                self.reject_next_motion = False
                return -40001, "", False
            if self.mode != 5:
                return -1, "", False
            target = [float(v) for v in re.findall(r"-?\d+(?:\.\d+)?", command.split("(", 1)[1])[:6]]
            joint = name == "JointMovJ" or "joint=" in command
            self.command_id += 1
            self.pending = (time.monotonic() + self.duration, joint, target)
            self.mode = 7
            if self.drop_next_motion_ack:
                self.drop_next_motion_ack = False
                return 0, "", True
            return 0, str(self.command_id) if self.protocol == "v4" else "", False
        return 0, "", False

    def packet(self):
        if self.pending and time.monotonic() >= self.pending[0]:
            _, joint, target = self.pending
            if not self.wrong_target:
                if joint:
                    self.joints = target
                    if self.ik_joints is not None and target == self.ik_joints:
                        self.pose = self.ik_pose.copy()
                else:
                    self.pose = target
            self.pending, self.mode = None, 5
        dtype = V3Type if self.protocol == "v3" else MyType
        p = np.zeros(1, dtype=dtype)
        p["len"] = 1440
        values = {"TestValue": 0 if self.bad_feedback else 0x0123456789ABCDEF,
                  "RobotMode": 9 if self.error else self.mode, "EnableStatus": self.mode != 4,
                  "ErrorStatus": self.error,
                  "RunningStatus": self.mode == 7, "RunQueuedCmd": self.mode != 4,
                  "QActual": self.joints, "QDActual": [1.] * 6 if self.mode == 7 else [0.] * 6,
                  "ToolVectorActual": self.pose, "CurrentCommandId": self.command_id}
        # Measured Nova2 hand values differ from signs of its J2/J3/J5 angles.
        values["HandType"] = [1, 1, -1, 0]
        legacy = {"TestValue": "test_value", "RobotMode": "robot_mode", "EnableStatus": "enable_status",
                  "RunningStatus": "running_status", "RunQueuedCmd": "run_queued_cmd",
                  "QActual": "q_actual", "QDActual": "qd_actual", "ToolVectorActual": "tool_vector_actual",
                  "ErrorStatus": "error_status", "HandType": "hand_type"}
        for key, value in values.items():
            if self.protocol == "v3":
                if key not in legacy:
                    continue
                key = legacy[key]
            p[key] = value
        return p.tobytes()

    def close(self):
        self.closed.set()
        for s in self.servers:
            s.shutdown()
            s.server_close()
