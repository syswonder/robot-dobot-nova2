# Hangkai TG-9801 gripper for Robonix

Standalone PC USB-RS485 primitive, without ROS, JAKA or Dobot SDK dependencies.
Supports Modbus RTU and the vendor custom protocol; optional Windows COM relay
for WSL. All eight public contracts are typed MCP RPCs in
`robonix/primitive/tg9801/*`, with gRPC lifecycle. No ROS state stream or standard
gripper-width contract is claimed.

```bash
rbnx validate primitives/tg9801_gripper
rbnx build -p primitives/tg9801_gripper
primitives/tg9801_gripper/rbnx-build/venv/bin/python -m unittest discover -s primitives/tg9801_gripper/tests -v
```

When installed as part of this Nova2 body, `rbnx boot` builds it automatically
on first use. Package dependencies stay in `rbnx-build/venv`. The body selects
COM3, 115200 baud, ID 1, Modbus; standalone defaults use the vendor's 1 Mbps and
read-only protocol detection. Neither changes the device's serial settings.

See [CAPABILITY.md](CAPABILITY.md), [config.spec](config.spec), and the
[deployment README](../../README.md) for units, limits, and startup.

New gripper implementation: MIT, see LICENSE. The existing WSL byte relay is
MulanPSL-2.0, see LICENSE-WINDOWS-SERIAL and ../../THIRD_PARTY.md.
