#!/usr/bin/env python3
"""Direct TG9801 read-only probe. Run only when its provider/GUI is stopped."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "primitives/tg9801_gripper"))
from tg9801.controller import Controller


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--windows-port", default="")
    parser.add_argument("--baudrate", type=int, default=115200)
    parser.add_argument("--device-id", type=int, default=1)
    parser.add_argument("--protocol", choices=("auto", "modbus", "custom"), default="auto")
    args = parser.parse_args()
    controller = Controller(dict(port=args.port, windows_port=args.windows_port,
                                 baudrate=args.baudrate, device_id=args.device_id,
                                 protocol=args.protocol, motion_enabled=False))
    try:
        controller.start()
        print(json.dumps(dict(info=controller.info(), state=controller.state()), ensure_ascii=False, indent=2))
    finally:
        controller.close()


if __name__ == "__main__":
    main()
