from __future__ import annotations

import asyncio
import json
from pathlib import Path
import sys
import time
import unittest

PKG = Path(__file__).resolve().parents[1]
for path in (PKG, PKG/"rbnx-build/codegen/proto_gen", PKG/"rbnx-build/codegen/robonix_mcp_types"):
    sys.path.insert(0, str(path))

from tg9801.controller import Controller
from tg9801.protocol import LinkError, custom_frame, modbus_frame
from fake_gripper import FakeGripper


class GripperTests(unittest.TestCase):
    def setUp(self):
        self.devices = []
        self.controllers = []

    def tearDown(self):
        for controller in self.controllers:
            controller.close()
        for device in self.devices:
            device.close()

    def connect(self, wire_protocol="modbus", **config):
        device = FakeGripper(wire_protocol)
        self.devices.append(device)
        controller = Controller(device.config | config)
        self.controllers.append(controller)
        controller.start()
        return device, controller

    def test_official_golden_frames(self):
        self.assertEqual(modbus_frame(1, 3, 6, 1).hex(), "010300060001640b")
        self.assertEqual(modbus_frame(1, 0x10, 15, [1000, 1000]).hex(), "0110000f00020403e803e83321")
        self.assertEqual(custom_frame(1, 8).hex(), "a3b40201080b")
        self.assertEqual(custom_frame(1, 3, b"\0\xe8\x03").hex(), "a3b405010300e803f4")

    def test_start_and_default_updates_are_read_only(self):
        device, controller = self.connect()
        controller.parameters(250, 10)
        self.assertEqual(device.writes, [])
        self.assertEqual(controller.state()["position_raw"], 500)
        self.assertEqual(controller.info()["default_force_percent"], 10)

    def test_modbus_grip_release_and_feedback(self):
        device, controller = self.connect()
        result = controller.command("grip", 250, 10)
        self.assertEqual(result["acknowledgement"], "device_acknowledged")
        self.assertEqual(device.writes, [(14, [0]), (18, [10]), (10, [250, 1])])
        self.assertTrue(controller.wait_for_state("holding", 1)["matched"])
        controller.command("open", 300)
        self.assertEqual(device.writes[-1], (10, [300, 0]))
        self.assertFalse(controller.state()["holding"])

    def test_custom_status_mapping_and_unacknowledged_motion(self):
        device, controller = self.connect("custom")
        result = controller.command("close", 250, 10)
        self.assertEqual(result["acknowledgement"], "host_sent_only")
        self.assertTrue(controller.state()["holding"])
        device.regs[6] = 2
        self.assertFalse(controller.state()["holding"])
        self.assertTrue(controller.state()["moving"])
        controller.command("open")
        time.sleep(.05)
        self.assertEqual(device.writes[-1], (1, []))

    def test_explicit_adaptive_status_schemes_do_not_confuse_waiting_and_holding(self):
        for scheme in ("modbus_manual", "three_is_holding"):
            device, controller = self.connect(adaptive_status_scheme=scheme)
            for status in range(4):
                device.regs[6] = status
                state = controller.state()
                self.assertEqual(state["status_raw"], status)
                self.assertEqual(state["holding"], status == (2 if scheme == "modbus_manual" else 3))
                self.assertEqual(state["moving"], status == 1 or (scheme == "three_is_holding" and status == 2))
                self.assertEqual(state["slipping"], status == 3 if scheme == "modbus_manual" else None)
            # Force-position has its own independent state register.
            device.regs[14], device.regs[47] = 1, 1
            self.assertFalse(controller.state()["holding"])
            device.regs[47] = 2
            self.assertTrue(controller.state()["holding"])
            self.assertFalse(device.writes)

    def test_invalid_adaptive_status_scheme_rejected(self):
        with self.assertRaisesRegex(ValueError, "adaptive_status_scheme"):
            Controller({"adaptive_status_scheme": "auto"})

    def test_custom_extended_payload_and_strict_response_identity(self):
        device, controller = self.connect("custom")
        device.extended = True
        self.assertEqual(controller.state()["custom_extension_hex"], "37000000")
        for field in ("wrong_id", "wrong_command", "bad_crc"):
            with self.subTest(field=field):
                setattr(device, field, True)
                with self.assertRaises(LinkError):
                    controller.state()
                setattr(device, field, False)

    def test_modbus_crc_wrong_id_and_exception_rejected(self):
        device, controller = self.connect()
        for field in ("wrong_id", "bad_crc"):
            setattr(device, field, True)
            with self.assertRaises(LinkError):
                controller.state()
            setattr(device, field, False)
        device.exception_address = 6
        with self.assertRaisesRegex(LinkError, "exception 0x02"):
            controller.state()

    def test_lost_ack_never_retried_and_all_further_writes_blocked(self):
        device, controller = self.connect()
        device.drop_ack = True
        with self.assertRaisesRegex(LinkError, "uncertain"):
            controller.command("close")
        self.assertEqual(len(device.writes), 1)
        for action in ("close", "open"):
            with self.assertRaisesRegex(LinkError, "blocked"):
                controller.command(action)
        self.assertEqual(len(device.writes), 1)
        self.assertTrue(controller.state()["write_fault"])

    def test_invalid_parameters_and_disabled_position_do_not_write(self):
        device, controller = self.connect()
        for args in ((199, 10), (1501, 10), (300, 51), (True, 10), (300, float("nan"))):
            with self.assertRaises(ValueError):
                controller.command("grip", *args)
        with self.assertRaisesRegex(LinkError, "disabled"):
            controller.command("move_position", position_raw=500)
        self.assertFalse(device.writes)

    def test_read_only_mode_and_error_flags_refuse_motion(self):
        device, controller = self.connect(motion_enabled=False)
        with self.assertRaisesRegex(LinkError, "read-only"):
            controller.command("close")
        controller.config["motion_enabled"] = True
        device.regs[35] = 8
        with self.assertRaisesRegex(LinkError, "error flags"):
            controller.command("close")
        self.assertFalse(device.writes)

    def test_bad_feedback_not_used_as_target(self):
        device, controller = self.connect()
        for addr, value in ((8, 1011), (6, 4), (14, 2), (18, 101)):
            old = device.regs[addr];device.regs[addr] = value
            with self.assertRaises(LinkError):
                controller.command("grip")
            device.regs[addr] = old
        self.assertFalse(device.writes)

    def test_measured_position_allows_small_endpoint_overshoot(self):
        device, controller = self.connect()
        device.regs[8] = 1002
        self.assertEqual(controller.state()["position_raw"], 1002)
        with self.assertRaises(ValueError):
            controller.command("move_position", position_raw=1002)
        self.assertFalse(device.writes)

    def test_force_position_uses_atomic_speed_target_and_measured_position(self):
        device, controller = self.connect(position_control_enabled=True)
        controller.command("move_position", 250, 10, 650)
        self.assertEqual(device.writes[-1], (15, [250, 650]))
        state = controller.state()
        self.assertEqual(state["position_raw"], 650)
        self.assertEqual(state["state"], "at_position")
        self.assertFalse(state["holding"])

    def test_wait_timeout_does_not_reverse_or_stop(self):
        device, controller = self.connect()
        with self.assertRaisesRegex(LinkError, "timed out"):
            controller.wait_for_state("holding", .1)
        self.assertFalse(device.writes)

    def test_auto_detection_only_reads_and_selects_protocol(self):
        for protocol in ("modbus", "custom"):
            device, controller = self.connect(protocol, protocol="auto")
            self.assertEqual(controller.info()["protocol"], protocol)
            self.assertFalse(device.writes)

    def test_typed_mcp_uses_same_transport_and_rejects_invalid_requests(self):
        import tg9801.driver as driver
        from tg9801_mcp import Close_Request, GetState_Request
        device, controller = self.connect()
        driver.controller = controller
        try:
            response = asyncio.run(driver.close_gripper(Close_Request(speed=250, force_percent=10)))
            self.assertTrue(response.ok, response.message)
            response = asyncio.run(driver.get_state(GetState_Request()))
            self.assertTrue(json.loads(response.result_json)["holding"])
            response = asyncio.run(driver.close_gripper(Close_Request(speed=199)))
            self.assertFalse(response.ok)
        finally:
            driver.controller = None


if __name__ == "__main__":
    unittest.main()
