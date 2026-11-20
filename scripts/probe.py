#!/usr/bin/env python3
"""Read controller state and calculate FK/IK without enabling or moving."""
from pathlib import Path
import json
import sys

import yaml

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "primitives/nova2_arm"))
from nova2_arm.controller import Controller


def main():
    manifest = yaml.safe_load((ROOT / "robonix_manifest.yaml").read_text())
    config = next(p["config"] for p in manifest["primitive"] if p["name"] == "nova2_arm")
    controller = Controller(config)
    try:
        controller.start()
        state = controller.state()
        print(json.dumps(state, indent=2, ensure_ascii=False))
        if state["dashboard_errors"]:
            raise SystemExit("Controller queries failed; see dashboard_errors above")
        pose = controller.forward_kinematics([0, -20, 40, 0, 40, 0])
        print("FK calculation only:", json.dumps(pose))
        print("IK calculation only:", json.dumps(controller.inverse_kinematics(pose["pose_mm_deg"])))
    finally:
        controller.close()


if __name__ == "__main__":
    main()
