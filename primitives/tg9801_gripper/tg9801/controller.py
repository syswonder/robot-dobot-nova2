"""Validated gripper operations. Startup and state queries are read-only."""
from __future__ import annotations

import math
import threading
import time

from .protocol import Link, LinkError


DEFAULTS = dict(port="/dev/ttyUSB0", windows_port="", protocol="auto", baudrate=1000000,
                device_id=1, timeout_s=.5, default_speed=300, default_force_percent=20,
                max_speed=1500, max_force_percent=50, motion_enabled=True,
                position_control_enabled=False, adaptive_status_scheme="modbus_manual")


def integer(name, value, low, high):
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise ValueError(f"{name} must be an integer in [{low}, {high}]")
    return value


class Controller:
    def __init__(self, config):
        unknown = set(config) - set(DEFAULTS)
        if unknown:
            raise ValueError("Unknown config keys: " + ", ".join(sorted(unknown)))
        self.config = DEFAULTS | config
        c = self.config
        if c["protocol"] not in ("auto", "modbus", "custom"):
            raise ValueError("protocol must be auto, modbus or custom")
        if c["adaptive_status_scheme"] not in ("modbus_manual", "three_is_holding"):
            raise ValueError("adaptive_status_scheme must be modbus_manual or three_is_holding")
        integer("device_id", c["device_id"], 1, 247)
        integer("baudrate", c["baudrate"], 9600, 1000000)
        integer("max_speed", c["max_speed"], 200, 1500)
        integer("max_force_percent", c["max_force_percent"], 1, 100)
        integer("default_speed", c["default_speed"], 200, c["max_speed"])
        integer("default_force_percent", c["default_force_percent"], 1, c["max_force_percent"])
        if isinstance(c["timeout_s"], bool) or not isinstance(c["timeout_s"], (int, float)) or not math.isfinite(c["timeout_s"]) or not .05 <= c["timeout_s"] <= 2:
            raise ValueError("timeout_s must be finite in [0.05, 2]")
        for name in ("motion_enabled", "position_control_enabled"):
            if not isinstance(c[name], bool):
                raise ValueError(name + " must be boolean")
        for name in ("port", "windows_port"):
            if not isinstance(c[name], str):
                raise ValueError(name + " must be a string")
        self.link = None
        self.lock = threading.RLock()
        self.fault = ""
        self.last_command = None
        self.speed = c["default_speed"]
        self.force = c["default_force_percent"]

    def start(self):
        with self.lock:
            if self.link is not None:
                return
            candidate = Link(self.config)
            try:
                candidate.read_state()
            except BaseException:
                candidate.close()
                raise
            self.link = candidate

    def close(self):
        with self.lock:
            if self.link:
                self.link.close()
                self.link = None

    def info(self):
        return dict(model="HKV TG-9801", connection="host USB-RS485; independent of Nova2",
                    protocol=self.link.protocol if self.link else self.config["protocol"],
                    device_id=self.config["device_id"], baudrate=self.config["baudrate"],
                    adaptive_status_scheme=self.config["adaptive_status_scheme"],
                    motion_enabled=self.config["motion_enabled"], speed_raw_range=[200, self.config["max_speed"]],
                    force_percent_range=[1, self.config["max_force_percent"]],
                    default_speed=self.speed, default_force_percent=self.force,
                    force_is_newtons=False,
                    position_unit="raw; command 0..1000, measured endpoint may reach 1010; opening direction uncalibrated",
                    position_control_enabled=self.config["position_control_enabled"],
                    absolute_width_mm_supported=False, stop_supported=False,
                    custom_open_speed_configurable=False,
                    startup="read-only; no release, grip, mode, limits or EEPROM writes")

    def _state(self):
        if self.link is None:
            raise LinkError("TG9801 is not active")
        state = self.link.read_state()
        state.update(sampled_at_unix_s=time.time(), write_fault=self.fault,
                     last_command=self.last_command, default_speed=self.speed,
                     default_force_percent=self.force)
        return state

    def state(self):
        with self.lock:
            return self._state()

    def parameters(self, speed=0, force_percent=0):
        # Runtime defaults only. They do not change a held object's grip force.
        with self.lock:
            speed, force = self._parameters(speed, force_percent)
            self.speed, self.force = speed, force
            return dict(default_speed=speed, default_force_percent=force,
                        hardware_written=False, effective="next grip/close/move_position; speed also next Modbus open")

    def _parameters(self, speed, force):
        integer("speed", speed, 0, self.config["max_speed"])
        integer("force_percent", force, 0, self.config["max_force_percent"])
        speed, force = speed or self.speed, force or self.force
        integer("speed", speed, 200, self.config["max_speed"])
        integer("force_percent", force, 1, self.config["max_force_percent"])
        return speed, force

    def command(self, action, speed=0, force_percent=0, position_raw=None):
        with self.lock:
            speed, force = self._parameters(speed, force_percent)
            if action not in ("open", "grip", "close", "move_position"):
                raise ValueError("Unknown action")
            if action == "move_position":
                integer("position_raw", position_raw, 0, 1000)
                if not self.config["position_control_enabled"]:
                    raise LinkError("Raw position control disabled; confirm firmware, direction and endpoints first")
            if not self.config["motion_enabled"]:
                raise LinkError("motion_enabled=false; read-only deployment")
            if self.fault:
                raise LinkError("Writes blocked after uncertain command; inspect hardware and restart provider: " + self.fault)
            before = self._state()  # Refuse blind commands when readback is lost.
            if before["error_flags"]:
                raise LinkError(f"Gripper reports error flags {before['error_flags']}")
            if before["moving"] and action != "open":
                raise LinkError("Gripper still moving; query state before another closing/position command")
            try:
                self.link.set_mode(1 if action == "move_position" else 0)
                if action != "open":
                    self.link.set_force(force)
                if action == "move_position":
                    self.link.move_position(speed, position_raw)
                else:
                    self.link.adaptive(action != "open", speed)
            except Exception as error:
                self.fault = str(error)
                raise LinkError("Command outcome uncertain; no retry, further writes blocked: " + self.fault) from error
            result = dict(action=action, speed_raw=speed, force_percent=force if action != "open" else None,
                          position_raw=position_raw, protocol=self.link.protocol,
                          acknowledgement="device_acknowledged" if self.link.protocol == "modbus" else "host_sent_only",
                          completion="unverified; query get_state or wait_for_state",
                          open_speed_applied=action == "open" and self.link.protocol == "modbus")
            self.last_command = result.copy()
            return result

    def wait_for_state(self, condition, timeout_s=5):
        if condition not in ("holding", "idle", "not_moving"):
            raise ValueError("condition must be holding, idle or not_moving")
        if isinstance(timeout_s, bool) or not isinstance(timeout_s, (int, float)) or not math.isfinite(timeout_s) or not 0 < timeout_s <= 10:
            raise ValueError("timeout_s must be finite in (0, 10]")
        deadline = time.monotonic() + timeout_s
        matches = 0
        while True:
            state = self.state()
            if state["write_fault"] or state["error_flags"]:
                raise LinkError("Cannot verify completion: gripper has a fault")
            match = state["holding"] if condition == "holding" else state["state"] == "idle" if condition == "idle" else not state["moving"]
            matches = matches + 1 if match else 0
            if matches >= 3:
                return dict(condition=condition, matched=True, state=state,
                            meaning="device state observed; idle/not_moving does not prove fully open or successful grasp")
            if time.monotonic() >= deadline:
                raise LinkError(f"Wait timed out for {condition}; last state={state['state']}. No stop or reverse sent")
            time.sleep(.1)
