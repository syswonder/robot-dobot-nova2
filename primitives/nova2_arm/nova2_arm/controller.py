"""Single-motion controller using version-matched feedback to verify completion."""
from __future__ import annotations

from collections import OrderedDict
import copy
import math
from pathlib import Path
import threading
import time
import uuid
import warnings
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

from .protocol import ControllerError, ControlAPI, Feedback, numbers


DEFAULTS = {
    "robot_ip": "192.168.5.1", "protocol": "v3", "dashboard_port": 29999, "motion_port": 30003, "feedback_port": 30004,
    "socket_timeout_s": 3.0, "feedback_timeout_s": 2.0, "motion_timeout_s": 120.0,
    "default_velocity_percent": 10, "max_velocity_percent": 20,
    "acceleration_percent": 10,
}
ACTIVE = {"PENDING", "RUNNING"}
MODE_NAMES = {1: "initializing", 2: "brake_open", 3: "powered_off", 4: "disabled",
              5: "enabled_idle", 6: "drag", 7: "running", 8: "single_move",
              9: "error", 10: "paused", 11: "collision"}


def vector(value, count: int, label: str) -> list[float]:
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise ValueError(f"{label} must contain exactly {count} numbers")
    if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in value):
        raise ValueError(f"{label} must contain only finite numbers")
    return [float(v) for v in value]


def integer(value, lower: int, upper: int, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not lower <= value <= upper:
        raise ValueError(f"{label} must be an integer in {lower}..{upper}")
    return value


def quaternion_to_euler(quaternion_xyzw) -> list[float]:
    q = vector(quaternion_xyzw, 4, "quaternion_xyzw")
    if abs(np.linalg.norm(q) - 1) > .01:
        raise ValueError("quaternion_xyzw must be normalized within 0.01")
    with warnings.catch_warnings():
        warnings.simplefilter("error", UserWarning)
        try:
            return Rotation.from_quat(q).as_euler("xyz", degrees=True).tolist()
        except UserWarning as error:
            raise ValueError("Quaternion is at an Euler singularity; use an explicit controller-native Rx/Ry/Rz pose") from error


class Controller:
    def __init__(self, config=None):
        config = dict(config or {})
        if set(config) - set(DEFAULTS):
            raise ValueError(f"Unknown configuration fields: {sorted(set(config) - set(DEFAULTS))}")
        self.config = {**DEFAULTS, **config}
        c = self.config
        if not isinstance(c["robot_ip"], str) or not c["robot_ip"].strip():
            raise ValueError("robot_ip must be nonempty")
        if c["protocol"] not in ("v3", "v4"):
            raise ValueError("protocol must be v3 or v4; Nova2 firmware 3.5.8.5 uses v3")
        for name in ("dashboard_port", "motion_port", "feedback_port"):
            integer(c[name], 1, 65535, name)
        for name in ("socket_timeout_s", "feedback_timeout_s", "motion_timeout_s"):
            value = c[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= 600:
                raise ValueError(f"{name} must be finite and in (0,600]")
        for name in ("default_velocity_percent", "max_velocity_percent", "acceleration_percent"):
            integer(c[name], 1, 100, name)
        if c["default_velocity_percent"] > c["max_velocity_percent"]:
            raise ValueError("Default speed must not exceed configured maximum")
        self.dashboard = ControlAPI(c)
        self.feedback = Feedback(c["robot_ip"], c["feedback_port"], c["feedback_timeout_s"], c["protocol"])
        self._lock = threading.RLock()
        self._closing = threading.Event()
        self._thread = None
        self._runs = OrderedDict()
        self._current = ""
        self._fault = ""
        root = ET.parse(Path(__file__).parent / "nova2_limits.urdf").getroot()
        self._limits = [(math.degrees(float(j.find("limit").get("lower"))),
                         math.degrees(float(j.find("limit").get("upper"))))
                        for j in root.findall("joint") if j.get("type") == "revolute"]
        if len(self._limits) != 6:
            raise ValueError("Expected six official Nova2 joint limits")

    def start(self):
        self.feedback.start()

    def _active(self):
        run = self._runs.get(self._current)
        return run if run and run["state"] in ACTIVE else None

    def _idle(self, s):
        # RunQueuedCmd reports the queue engine, which stays on while enabled and idle.
        return (s["mode"] in (4, 5) and not s["running"]
                and not s["paused"] and np.isfinite(s["joint_velocity_deg_s"]).all()
                and max(abs(v) for v in s["joint_velocity_deg_s"]) < .5)

    def _require_idle(self):
        s = self.feedback.snapshot()
        mode = int(numbers(self.dashboard.call("RobotMode"), 1)[0])
        if self._fault:
            raise ControllerError(f"Previous motion outcome needs inspection: {self._fault}. Use stop to confirm an idle state before another move.")
        if self._active() or mode != 5 or s["mode"] != 5 or not s["enabled"] or s["error"] or s["collision"] or not self._idle(s):
            raise ControllerError(f"Motion requires enabled, idle, error-free robot with no active motion from this provider; mode={mode}")
        return s

    def _wait_mode(self, modes, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            s = self.feedback.snapshot()
            if s["mode"] in modes and self._idle(s):
                return s
            if s["error"] or s["collision"]:
                raise ControllerError("Controller reports error/collision; inspect get_state")
            time.sleep(.05)
        raise ControllerError("Controller accepted the command but feedback did not confirm the requested mode")

    def state(self):
        with self._lock:
            s = self.feedback.snapshot()
            s.update(mode_name=MODE_NAMES.get(s["mode"], "unknown"), robot_ip=self.config["robot_ip"], protocol=self.config["protocol"],
                     fault=self._fault, motion=self.motion_status(""), user=0, tool=0,
                     joint_limits_deg=self._limits)
            errors = []
            for name, args, kwargs, key, count in [
                ("GetAngle", (), {}, "joints_deg", 6),
                ("GetPose", (), {"user": 0, "tool": 0}, "pose_mm_deg", 6),
                ("RobotMode", (), {}, "dashboard_mode", 1),
            ]:
                try:
                    s[key] = numbers(self.dashboard.call(name, *args, **kwargs), count)
                except ControllerError as error:
                    errors.append(str(error))
            try:
                s["alarm_ids"] = self.dashboard.call("GetErrorID")
            except ControllerError as error:
                errors.append(str(error))
            if "pose_mm_deg" in s:
                s["quaternion_xyzw"] = Rotation.from_euler("xyz", s["pose_mm_deg"][3:], degrees=True).as_quat().tolist()
            s["dashboard_errors"] = list(dict.fromkeys(errors))
            return s

    def enable(self):
        with self._lock:
            if self._active():
                raise ControllerError("A motion is active")
            s = self.feedback.snapshot()
            if s["error"] or s["collision"] or not self._idle(s):
                raise ControllerError("Enable requires an idle robot with errors resolved")
            self.dashboard.call("EnableRobot")
            return self._wait_mode({5})

    def stop(self):
        with self._lock:
            try:
                self.dashboard.call("Stop")
                self._wait_mode({4, 5}, timeout=5)
            except ControllerError as error:
                self._fault = str(error)
                if self._active():
                    self._active().update(state="UNKNOWN", message=f"Stop not confirmed: {error}")
                raise
            if self._active():
                self._active().update(state="CANCELED", message="Stop acknowledged; feedback confirms idle")
            self._fault = ""
            return {"stopped": True, "motion": self.motion_status("")}

    def disable(self):
        with self._lock:
            if self._active() or not self._idle(self.feedback.snapshot()):
                try:
                    self.stop()
                except ControllerError:
                    # An alarm can mask idle mode; it must not prevent explicit disable.
                    pass
            self.dashboard.call("DisableRobot")
            deadline = time.monotonic() + 8
            while time.monotonic() < deadline:
                s = self.feedback.snapshot()
                if (not s["enabled"] and not s["running"]
                        and np.isfinite(s["joint_velocity_deg_s"]).all()
                        and max(abs(v) for v in s["joint_velocity_deg_s"]) < .5):
                    if self._active():
                        self._active().update(state="CANCELED", message="Disable confirmed by feedback")
                    return s
                time.sleep(.05)
            raise ControllerError("Disable accepted but disabled/stationary feedback was not confirmed")

    def clear_error(self):
        with self._lock:
            if self._active():
                raise ControllerError("Stop active motion before clearing errors")
            self.dashboard.call("ClearError")
            return {"clear_requested": True, "feedback": self.feedback.snapshot(),
                    "alarm_ids": self.dashboard.call("GetErrorID"),
                    "message": "Check remaining alarm IDs. This does not enable or resume the robot."}

    def _joints(self, values):
        values = vector(values, 6, "joints_deg")
        for index, (v, (lower, upper)) in enumerate(zip(values, self._limits), 1):
            if not lower <= v <= upper:
                raise ValueError(f"J{index}={v} is outside official URDF range {lower:.3f}..{upper:.3f} degrees")
        return values

    def forward_kinematics(self, joints_deg):
        with self._lock:
            return {"pose_mm_deg": numbers(self.dashboard.call("PositiveKin", *self._joints(joints_deg), user=0, tool=0), 6), "user": 0, "tool": 0}

    def inverse_kinematics(self, pose_mm_deg):
        with self._lock:
            pose = vector(pose_mm_deg, 6, "pose_mm_deg")
            near = numbers(self.dashboard.call("GetAngle"), 6)
            return {"joints_deg": self._inverse(pose, near), "user": 0, "tool": 0}

    def _inverse(self, pose, near):
        result = numbers(self.dashboard.call("InverseKin", *pose, user=0, tool=0,
                         useJointNear=1, JointNear="{" + ",".join(f"{v:.6f}" for v in near) + "}"), 6)
        return self._joints(result)

    def move_joints(self, joints_deg, velocity_percent=0):
        target = self._joints(joints_deg)
        speed = self.config["default_velocity_percent"] if velocity_percent == 0 else velocity_percent
        integer(speed, 1, self.config["max_velocity_percent"], "velocity_percent")
        return self._move("joints", target, "MovJ", velocity=speed)

    def move_pose(self, pose_mm_deg, motion="MovJ", velocity_percent=0):
        target = vector(pose_mm_deg, 6, "pose_mm_deg")
        if motion not in ("MovJ", "MovL"):
            raise ValueError("motion must be exactly MovJ or MovL")
        speed = self.config["default_velocity_percent"] if velocity_percent == 0 else velocity_percent
        integer(speed, 1, self.config["max_velocity_percent"], "velocity_percent")
        return self._move("pose", target, motion, velocity=speed)

    def move_quaternion(self, position_mm, quaternion_xyzw, motion="MovJ", velocity_percent=0):
        return self.move_pose(vector(position_mm, 3, "position_mm") + quaternion_to_euler(quaternion_xyzw),
                              motion, velocity_percent)

    def _move(self, kind, target, motion, velocity):
        with self._lock:
            idle_state = self._require_idle()
            joint_solution = None
            if kind == "pose":
                joint_solution = self._inverse(target, numbers(self.dashboard.call("GetAngle"), 6))
            if self.config["protocol"] == "v3":
                self.dashboard.call("CP", 0)
            run_id = uuid.uuid4().hex
            run = {"run_id": run_id, "state": "PENDING", "kind": kind, "target": target,
                   "motion": motion, "command_id": None, "message": "Submitting motion", "elapsed_s": 0.0}
            self._runs[run_id] = run
            self._current = run_id
            while len(self._runs) > 100:
                self._runs.popitem(last=False)
            kwargs = {"user": 0, "tool": 0, "a": self.config["acceleration_percent"], "cp": 0}
            kwargs["v"] = velocity
            try:
                wire_target = target
                joint_mode = kind == "joints"
                if self.config["protocol"] == "v3" and kind == "pose":
                    if motion == "MovJ":
                        # Execute the selected nearby IK solution rather than a saved hand configuration.
                        wire_target, joint_mode = joint_solution, True
                    else:
                        self.dashboard.prepare_linear(idle_state["hand_type"])
                payload = self.dashboard.call(motion, *wire_target, coordinateMode=1 if joint_mode else 0, **kwargs)
                if self.config["protocol"] == "v4" and not payload.strip().isdigit():
                    raise ControllerError(f"Motion acknowledged without a valid command ID: {payload!r}")
                run.update(command_id=int(payload) if self.config["protocol"] == "v4" else None,
                           state="RUNNING", message="Accepted; waiting for feedback and target verification")
            except ControllerError as error:
                # A known controller rejection is definite; a failed TCP exchange is not.
                uncertain = not str(error).startswith("Dobot error ")
                run.update(state="UNKNOWN" if uncertain else "FAILED", message=str(error))
                if uncertain:
                    self._fault = str(error)
                    try:
                        self.dashboard.call("Stop")
                        self._wait_mode({4, 5}, timeout=5)
                        run["message"] += "; Stop confirmed; inspect the robot before retrying"
                    except ControllerError as stop_error:
                        run["message"] += f"; Stop not confirmed: {stop_error}"
                raise
            self._thread = threading.Thread(target=self._monitor, args=(run_id,), daemon=True, name="nova2-motion")
            self._thread.start()
            return copy.deepcopy(run)

    def _at_target(self, run, s):
        if run["kind"] == "joints":
            return max(abs(a - b) for a, b in zip(s["joints_deg"], run["target"])) <= .5
        pose = numbers(self.dashboard.call("GetPose", user=0, tool=0), 6)
        target = run["target"]
        distance = np.linalg.norm(np.asarray(pose[:3]) - target[:3])
        angle = (Rotation.from_euler("xyz", pose[3:], degrees=True).inv()
                 * Rotation.from_euler("xyz", target[3:], degrees=True)).magnitude()
        return distance <= 1.0 and math.degrees(angle) <= 1.0

    def _monitor(self, run_id):
        started = time.monotonic()
        settled = 0
        while not self._closing.wait(.05):
            with self._lock:
                run = self._runs[run_id]
                if run["state"] not in ACTIVE:
                    return
                run["elapsed_s"] = time.monotonic() - started
                try:
                    s = self.feedback.snapshot()
                    if s["error"] or s["collision"] or not s["enabled"] or s["mode"] not in (5, 7):
                        raise ControllerError(f"Motion interrupted by robot mode {s['mode']}")
                    if run["elapsed_s"] > self.config["motion_timeout_s"]:
                        raise ControllerError("Motion exceeded configured deadline")
                    if self.config["protocol"] == "v4" and s["command_id"] > run["command_id"]:
                        raise ControllerError("Another controller command superseded this motion")
                    matched = self.config["protocol"] == "v3" or s["command_id"] == run["command_id"]
                    reached = matched and run["elapsed_s"] >= .25 and self._idle(s) and self._at_target(run, s)
                    settled = settled + 1 if reached else 0
                    if settled >= 3:
                        run.update(state="SUCCEEDED", message="Idle feedback and actual target verified" + ("; command ID matched" if self.config["protocol"] == "v4" else "; V3 has no command ID"))
                        return
                    if self._idle(s) and matched and run["elapsed_s"] > 2 and not reached:
                        raise ControllerError("Controller became idle without reaching the target")
                except (ControllerError, ValueError) as error:
                    reason = str(error)
                    try:
                        self.stop()
                        run.update(state="FAILED", message=reason + "; Stop confirmed")
                    except ControllerError as stop_error:
                        self._fault = reason + "; " + str(stop_error)
                        run.update(state="UNKNOWN", message=self._fault)
                    return

    def motion_status(self, run_id="", wait_s=0):
        if isinstance(wait_s, bool) or not isinstance(wait_s, (int, float)) or not math.isfinite(wait_s) or not 0 <= wait_s <= 10:
            raise ValueError("wait_s must be in 0..10 seconds")
        deadline = time.monotonic() + wait_s
        while True:
            with self._lock:
                key = run_id or self._current
                run = self._runs.get(key)
                if run is None:
                    return {"run_id": key, "state": "IDLE" if not key else "NOT_FOUND", "message": "No matching motion"}
                result = copy.deepcopy(run)
            if result["state"] not in ACTIVE or time.monotonic() >= deadline:
                return result
            time.sleep(.05)

    def close(self):
        with self._lock:
            if self._active():
                try:
                    self.stop()
                except ControllerError:
                    pass
            self._closing.set()
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=2)
        self.feedback.close()
        self.dashboard.close()
