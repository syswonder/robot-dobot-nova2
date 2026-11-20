#!/usr/bin/env python3
"""Bounded 1%-speed real Cartesian checks through the running Robonix stack."""
from __future__ import annotations

import argparse
import json

import numpy as np
from scipy.spatial.transform import Rotation

from test_hardware_motion import HardwareChecks, ROOT


class CartesianChecks(HardwareChecks):
    tcp_bound_mm = 30
    orientation_bound_deg = 20

    def guard_pose(self, pose):
        if np.linalg.norm(np.asarray(pose[:3]) - self.pose[:3]) > self.tcp_bound_mm:
            raise RuntimeError(f"Test TCP exceeds {self.tcp_bound_mm} mm from starting position")
        rotation = Rotation.from_euler("xyz", self.pose[3:], degrees=True).inv() * Rotation.from_euler("xyz", pose[3:], degrees=True)
        if np.degrees(rotation.magnitude()) > self.orientation_bound_deg:
            raise RuntimeError(f"Test orientation exceeds {self.orientation_bound_deg} degrees from start")

    def guard_joints(self, target):
        if max(abs(np.asarray(target) - self.joints)) > 26:
            raise RuntimeError("Test exceeds 26 degrees from starting joints")
        current = self.call("get_state")["joints_deg"]
        if max(abs(np.asarray(target) - current)) > 2.1:
            raise RuntimeError("Test exceeds a 2.1-degree joint step")
        pose = self.call("forward_kinematics", joints_deg=target)["pose_mm_deg"]
        self.guard_pose(pose)

    def prepare(self):
        if max(abs(np.asarray(self.joints))) > 1:
            raise RuntimeError("Preparation is only defined from the measured near-zero pose")
        self.tcp_bound_mm, self.orientation_bound_deg = 60, 30
        # Keep J6 away from its zero-quadrant boundary for native V3 MovL.
        offset = np.array([0, -7, 24, 0, 12, 2])
        # Check every joint waypoint before enabling; this is a local check, not collision planning.
        waypoints = [(np.asarray(self.joints) + offset * i / 12).tolist() for i in range(1, 13)]
        for target in waypoints:
            self.guard_pose(self.call("forward_kinematics", joints_deg=target)["pose_mm_deg"])
        end = self.call("forward_kinematics", joints_deg=waypoints[-1])["pose_mm_deg"]
        solution = self.call("inverse_kinematics", pose_mm_deg=end)["joints_deg"]
        if max(abs(np.asarray(solution) - waypoints[-1])) > .1:
            raise RuntimeError("Preparation IK does not match its joint target")
        self.call("enable")
        for i, target in enumerate(waypoints, 1):
            actual = self.move_joints(target)
            self.guard_pose(actual["pose_mm_deg"])
            print(f"PASS preparation {i}/12: {actual['joints_deg']}", flush=True)
        self.joints, self.pose = actual["joints_deg"], actual["pose_mm_deg"]
        self.tcp_bound_mm, self.orientation_bound_deg = 30, 20

    def cartesian(self, target, motion, quaternion=False):
        self.guard_pose(target)
        solution = self.call("inverse_kinematics", pose_mm_deg=target)["joints_deg"]
        self.guard_joints(solution)
        if quaternion:
            run = self.call("move_quaternion", position_mm=target[:3],
                            quaternion_xyzw=Rotation.from_euler("xyz", target[3:], degrees=True).as_quat().tolist(),
                            motion=motion, velocity_percent=1)
        else:
            run = self.call("move_pose", pose_mm_deg=target, motion=motion, velocity_percent=1)
        state = self.wait(run)
        self.guard_pose(state["pose_mm_deg"])
        error = float(np.linalg.norm(np.asarray(state["pose_mm_deg"][:3]) - target[:3]))
        angular_error = float(np.degrees((Rotation.from_euler("xyz", target[3:], degrees=True).inv() *
                                         Rotation.from_euler("xyz", state["pose_mm_deg"][3:], degrees=True)).magnitude()))
        if error > .2 or angular_error > .1:
            raise RuntimeError(f"Measured target error: {error:.6f} mm, {angular_error:.6f} degrees")
        print(f"PASS {'quaternion' if quaternion else 'pose'} {motion}: error {error:.6f} mm / {angular_error:.6f} deg", flush=True)

    def test(self):
        state = self.call("get_state")
        reference = state["pose_mm_deg"]
        self.call("inverse_kinematics", pose_mm_deg=reference)
        self.call("enable")
        # Keep all targets local and check native IK joint steps before sending.
        for axis, delta, motion in ((0, .5, "MovJ"), (1, -.1, "MovL"), (2, -.5, "MovL")):
            target = reference.copy()
            target[axis] += delta
            self.cartesian(target, motion)
            self.cartesian(reference, motion)
        for motion, axis, delta, rot_axis, angle in (("MovJ", 0, -.5, "z", .3), ("MovL", 2, -.5, "x", .2)):
            target = reference.copy()
            target[axis] += delta
            target[3:] = (Rotation.from_euler(rot_axis, angle, degrees=True) *
                          Rotation.from_euler("xyz", reference[3:], degrees=True)).as_euler("xyz", degrees=True).tolist()
            self.cartesian(target, motion, quaternion=True)
            self.cartesian(reference, motion, quaternion=True)
        print("PASS returned to the nonsingular reference pose", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", required=True, action="store_true")
    parser.add_argument("--prepare-from-near-zero", action="store_true",
                        help="Stage J2 -7, J3 +24, J5 +12, J6 +2 degrees from near-zero")
    parser.add_argument("--report", default=str(ROOT / ".artifacts/hardware-cartesian-nonsingular.json"))
    args = parser.parse_args()
    checks = CartesianChecks(args.report)
    try:
        if args.prepare_from_near_zero:
            checks.prepare()
        checks.test()
    finally:
        try:
            checks.call("stop")
        finally:
            checks.call("disable")
    print("PASS real Cartesian tests; arm disabled at nonsingular reference", flush=True)


if __name__ == "__main__":
    main()
