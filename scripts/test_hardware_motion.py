#!/usr/bin/env python3
"""Run bounded real-arm checks; --execute explicitly enables physical motion."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from check_runtime import ROOT, call_tools


class HardwareChecks:
    def __init__(self, report):
        self.report = Path(report)
        self.records = []
        self.baseline = self.call("get_state")
        self.joints = self.baseline["joints_deg"]
        self.pose = self.baseline["pose_mm_deg"]
        if self.baseline["error"] or self.baseline["running"] or self.baseline["motion"]["state"] in ("PENDING", "RUNNING"):
            raise RuntimeError("Robot must be stationary and error-free before testing")

    def call(self, name, **args):
        try:
            result = call_tools([(name, args)])[name]
        except Exception as error:
            self.records.append({"tool": name, "args": args, "error": str(error)})
            raise
        else:
            self.records.append({"tool": name, "args": args, "result": result})
            return result
        finally:
            self.report.parent.mkdir(exist_ok=True, parents=True)
            self.report.write_text(json.dumps(self.records, indent=2))

    def guard_joints(self, target):
        if max(abs(a-b) for a,b in zip(target, self.joints)) > 1.5:
            raise RuntimeError("Test target exceeds 1.5 degrees from initial joints")
        current = self.call("get_state")["joints_deg"]
        if max(abs(a-b) for a,b in zip(target, current)) > 1.5:
            raise RuntimeError("Test target exceeds a 1.5-degree step from measured joints")
        pose = self.call("forward_kinematics", joints_deg=target)["pose_mm_deg"]
        self.guard_pose(pose)

    def guard_pose(self, pose):
        if np.linalg.norm(np.asarray(pose[:3])-self.pose[:3]) > 10:
            raise RuntimeError("Test target exceeds 10 mm from the initial TCP")
        delta = Rotation.from_euler("xyz", self.pose[3:], degrees=True).inv() * Rotation.from_euler("xyz", pose[3:], degrees=True)
        if np.degrees(delta.magnitude()) > 3:
            raise RuntimeError("Test orientation exceeds 3 degrees from the initial TCP")

    def wait(self, run):
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            value = self.call("motion_status", run_id=run["run_id"], wait_s=1)
            if value["state"] == "SUCCEEDED":
                return self.call("get_state")
            if value["state"] not in ("PENDING", "RUNNING"):
                raise RuntimeError(json.dumps(value))
        self.call("stop")
        raise RuntimeError("Bounded test did not finish within 15 seconds")

    def move_joints(self, target):
        self.guard_joints(target)
        measured = self.wait(self.call("move_joints", joints_deg=target, velocity_percent=1))
        error = max(abs(a-b) for a,b in zip(measured["joints_deg"], target))
        if error > .15:
            raise RuntimeError(f"Measured joint error {error} exceeds 0.15 degrees")
        return measured

    def joints_test(self):
        self.call("disable")
        self.call("enable")
        for index in (5, 0, 1, 2, 3, 4):
            target = self.joints.copy()
            target[index] += .6
            actual = self.move_joints(target)
            print(f"PASS J{index+1}: +0.6 deg at 1%, measured {actual['joints_deg'][index]:.6f}", flush=True)
            self.move_joints(self.joints)
        self.call("stop")
        print("PASS stop and return to initial joint pose", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true", required=True)
    parser.add_argument("--report", default=str(ROOT / ".artifacts/hardware-joint-tests.json"))
    args = parser.parse_args()
    checks = HardwareChecks(args.report)
    try:
        checks.joints_test()
    finally:
        try:
            checks.call("stop")
        finally:
            checks.call("disable")
    print("PASS: physical joint checks; arm is disabled", flush=True)


if __name__ == "__main__":
    main()
