"""Official SDK commands with bounded, non-replaying TCP exchanges."""
from __future__ import annotations

import re
import socket
import threading
import time

import numpy as np

from .vendor.dobot_api import DobotApiDashboard, MyType
from .vendor.dobot_api_v3 import DobotApiDashboard as V3Dashboard, DobotApiMove as V3Move, MyType as V3Type


class ControllerError(RuntimeError):
    pass


def parse_reply(reply: str) -> str:
    match = re.fullmatch(r"\s*(-?\d+)\s*,\s*\{(.*?)\}\s*,\s*[A-Za-z_]\w*\(.*\);\s*", reply, re.S)
    if not match:
        raise ControllerError(f"Malformed controller reply: {reply!r}")
    code, payload = int(match[1]), match[2]
    if code:
        hint = " Switch the controller to TCP/IP mode and wait for startup." if code == -2 else ""
        raise ControllerError(f"Dobot error {code}: {payload}.{hint}")
    return payload


def numbers(payload: str, count: int) -> list[float]:
    try:
        result = [float(value) for value in payload.split(",")]
    except ValueError as error:
        raise ControllerError(f"Non-numeric controller payload: {payload}") from error
    if len(result) != count or not np.isfinite(result).all():
        raise ControllerError(f"Expected {count} finite controller values: {payload}")
    return result


class Transport:
    def __init__(self, ip: str, port: int, timeout: float):
        # The upstream transport retries indefinitely and can resend motion.
        # Keep its command builders but replace the complete transport.
        self.ip, self.port, self.timeout = ip, port, timeout
        self.socket_dobot = None
        self._io_lock = threading.Lock()
        self._broken = False

    def sendRecvMsg(self, command: str) -> str:
        with self._io_lock:
            if self._broken:
                raise ControllerError("Dashboard connection failed; restart the provider after inspecting the robot. No command was replayed.")
            try:
                if self.socket_dobot is None:
                    self.socket_dobot = socket.create_connection((self.ip, self.port), self.timeout)
                self.socket_dobot.settimeout(self.timeout)
                self.socket_dobot.sendall(command.encode("ascii"))
                data = bytearray()
                deadline = time.monotonic() + self.timeout
                while b";" not in data:
                    self.socket_dobot.settimeout(max(.001, deadline - time.monotonic()))
                    chunk = self.socket_dobot.recv(8192)
                    if not chunk:
                        raise OSError("controller closed the dashboard connection")
                    data.extend(chunk)
                    if len(data) > 65536 or time.monotonic() > deadline:
                        raise OSError("controller reply exceeded its size or time limit")
                reply = data.decode("utf-8").strip()
                # Verify the echo so stale or unrelated replies cannot acknowledge a motion.
                if not reply.endswith("," + command + ";"):
                    raise OSError(f"controller command echo mismatch: {reply!r}")
                return reply
            except (OSError, UnicodeError) as error:
                self._broken = True
                self.close()
                raise ControllerError(f"Dashboard exchange failed; command outcome may be unknown, do not retry motion blindly: {error}") from error

    def call(self, name: str, *args, **kwargs) -> str:
        return parse_reply(getattr(self, name)(*args, **kwargs))

    def close(self):
        sock, self.socket_dobot = self.socket_dobot, None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            sock.close()


class Dashboard(Transport, DobotApiDashboard):
    pass


class LegacyDashboard(Transport, V3Dashboard):
    pass


class LegacyMove(Transport, V3Move):
    pass


class ControlAPI:
    def __init__(self, config):
        self.protocol = config["protocol"]
        args = (config["robot_ip"], config["dashboard_port"], config["socket_timeout_s"])
        self.dashboard = (LegacyDashboard if self.protocol == "v3" else Dashboard)(*args)
        self.move = LegacyMove(config["robot_ip"], config["motion_port"], config["socket_timeout_s"]) if self.protocol == "v3" else self.dashboard

    def call(self, name, *args, **kwargs):
        if self.protocol == "v4":
            return self.dashboard.call(name, *args, **kwargs)
        if name == "GetPose":
            return parse_reply(self.dashboard.sendRecvMsg("GetPose(User=0,Tool=0)"))
        if name == "Stop":
            return self.dashboard.call("ResetRobot")
        if name == "PositiveKin":
            return self.dashboard.call("PositiveSolution", *args, 0, 0)
        if name == "InverseKin":
            # Upstream V3's variadic builder uses repr() without commas.
            # The native protocol takes positional isJointNear and JointNear.
            values = ",".join(f"{value:.6f}" for value in args)
            command = f"InverseSolution({values},0,0,1,{kwargs['JointNear']})"
            return parse_reply(self.dashboard.sendRecvMsg(command))
        if name in ("MovJ", "MovL"):
            method = "JointMovJ" if kwargs["coordinateMode"] == 1 else name
            params = [] if method == "JointMovJ" else ["User=0", "Tool=0"]
            if name == "MovJ":
                params += [f"SpeedJ={kwargs['v']}", f"AccJ={kwargs['a']}"]
            else:
                params += [f"SpeedL={kwargs['v']}", f"AccL={kwargs['a']}"]
            return self.move.call(method, *args, *params)
        return self.dashboard.call(name, *args, **kwargs)

    def prepare_linear(self, hand):
        if self.protocol != "v3":
            return
        # Nova2's actual hand configuration does not follow the generic SDK's
        # J2/J3/J5 sign rule. Preserve the controller-reported branch for MovL.
        if len(hand) != 4 or any(value not in (-1, 1) for value in hand[:3]):
            raise ControllerError(f"Invalid controller hand configuration: {hand}")
        # The upstream SDK defines SetArmOrientation twice, losing its four-argument form.
        parse_reply(self.dashboard.sendRecvMsg("SetArmOrientation(" + ",".join(map(str, hand)) + ")"))
        # A previous TCPSpeed setting would override SpeedL's percentage.
        self.dashboard.call("TCPSpeedEnd")

    def close(self):
        self.move.close()
        if self.move is not self.dashboard:
            self.dashboard.close()


class Feedback:
    def __init__(self, ip: str, port: int, timeout: float, protocol="v4"):
        self.ip, self.port, self.timeout = ip, port, timeout
        self.protocol = protocol
        self.dtype = V3Type if protocol == "v3" else MyType
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._sample = None
        self._received_at = 0.0
        self._error = "No feedback received"
        self._socket = None
        self._thread = None

    def start(self):
        if self.dtype.itemsize != 1440:
            raise ControllerError(f"Unexpected official feedback layout: {self.dtype.itemsize}")
        self._socket = socket.create_connection((self.ip, self.port), self.timeout)
        self._socket.settimeout(.25)
        self._thread = threading.Thread(target=self._read, daemon=True, name="nova2-feedback")
        self._thread.start()
        if not self._ready.wait(self.timeout):
            self.close()
            raise ControllerError("No valid feedback packet; check configured protocol, firmware/port and TCP/IP mode")
        self.snapshot()

    def _read(self):
        pending = bytearray()
        try:
            while not self._stop.is_set():
                try:
                    chunk = self._socket.recv(16384)
                except socket.timeout:
                    continue
                if not chunk:
                    raise OSError("controller closed the feedback connection")
                pending.extend(chunk)
                while len(pending) >= self.dtype.itemsize:
                    packet = np.frombuffer(bytes(pending[:1440]), dtype=self.dtype)[0]
                    magic = "test_value" if self.protocol == "v3" else "TestValue"
                    if int(packet["len"]) != 1440 or int(packet[magic]) != 0x0123456789ABCDEF:
                        del pending[0]
                        continue
                    del pending[:1440]
                    with self._lock:
                        self._sample = packet.copy()
                        self._received_at = time.monotonic()
                        self._error = ""
                    self._ready.set()
        except OSError as error:
            if not self._stop.is_set():
                with self._lock:
                    self._error = str(error)
        finally:
            self._ready.set()

    def snapshot(self) -> dict:
        with self._lock:
            age = time.monotonic() - self._received_at
            if self._sample is None or self._error or age > self.timeout:
                raise ControllerError(f"Feedback unavailable or stale: {self._error or f'{age:.2f} seconds old'}")
            p = self._sample
            if self.protocol == "v3":
                return {
                    "mode": int(p["robot_mode"]), "enabled": bool(p["enable_status"]),
                    "error": bool(p["error_status"]), "collision": int(p["robot_mode"]) == 11,
                    "running": bool(p["running_status"]), "queue_running": bool(p["run_queued_cmd"]),
                    "paused": bool(p["pause_cmd_flag"]), "command_id": None,
                    "joints_deg": p["q_actual"].tolist(), "joint_velocity_deg_s": p["qd_actual"].tolist(),
                    "tcp_pose_mm_deg": p["tool_vector_actual"].tolist(),
                    "hand_type": p["hand_type"].tolist(),
                    "feedback_user": int(p["user"]), "feedback_tool": int(p["tool"]),
                    "feedback_age_s": age,
                }
            return {
                "mode": int(p["RobotMode"]), "enabled": bool(p["EnableStatus"]),
                "error": bool(p["ErrorStatus"]), "collision": bool(p["CollisionState"]),
                "running": bool(p["RunningStatus"]), "queue_running": bool(p["RunQueuedCmd"]),
                "paused": bool(p["PauseCmdFlag"]), "command_id": int(p["CurrentCommandId"]),
                "joints_deg": p["QActual"].tolist(), "joint_velocity_deg_s": p["QDActual"].tolist(),
                "tcp_pose_mm_deg": p["ToolVectorActual"].tolist(),
                "hand_type": p["HandType"].tolist(),
                "feedback_user": int(p["User"]), "feedback_tool": int(p["Tool"]),
                "feedback_age_s": age,
            }

    def close(self):
        self._stop.set()
        if self._socket is not None:
            try:
                self._socket.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            self._socket.close()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=1)
