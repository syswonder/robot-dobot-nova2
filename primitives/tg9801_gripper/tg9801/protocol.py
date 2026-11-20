"""Small, bounded implementation of the official TG-9801 serial protocols.

Register/command definitions: https://www.hkvtech.cn/toolbox/UserManual.html#sdk
No automatic retransmission, protocol switching, EEPROM writes or calibration.
"""
from __future__ import annotations

import struct
import time

import serial


class LinkError(RuntimeError):
    pass


# Position commands use the documented 0..1000 range. Real TG-9801 firmware can
# report a few encoder counts past an end stop, so validate feedback separately.
MEASURED_POSITION_MAX = 1010


def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xA001 if crc & 1 else 0)
    return crc.to_bytes(2, "little")


def modbus_frame(device_id: int, function: int, address: int, values) -> bytes:
    if function == 0x10:
        data = struct.pack(">" + "H" * len(values), *values)
        body = struct.pack(">BBHHB", device_id, function, address, len(values), len(data)) + data
    else:
        body = struct.pack(">BBHH", device_id, function, address, values)
    return body + crc16(body)


def custom_frame(device_id: int, command: int, payload=b"") -> bytes:
    body = bytes([2 + len(payload), device_id, command]) + payload
    return b"\xa3\xb4" + body + bytes([sum(body) & 255])


class Link:
    def __init__(self, config):
        self.device_id = config["device_id"]
        self.timeout = config["timeout_s"]
        self.protocol = config["protocol"]
        # Explicit commissioning choice; never infer this from a live status.
        self.adaptive_status_scheme = config.get("adaptive_status_scheme", "modbus_manual")
        self.bridge = None
        self.serial = None
        try:
            port = config["port"]
            if config["windows_port"]:
                from .windows_serial import WindowsSerialBridge
                self.bridge = WindowsSerialBridge(config["windows_port"], config["baudrate"])
                port = self.bridge.port
            self.serial = serial.Serial(port, config["baudrate"], timeout=min(.05, self.timeout),
                                        write_timeout=self.timeout, exclusive=True)
            if self.protocol == "auto":
                failures = []
                for candidate in ("modbus", "custom"):
                    self.protocol = candidate
                    try:
                        self.read_state()
                        break
                    except (LinkError, OSError, ValueError) as error:
                        failures.append(f"{candidate}: {error}")
                else:
                    raise LinkError("Read-only protocol detection failed; " + "; ".join(failures))
        except BaseException:
            self.close()
            raise

    def close(self):
        if self.serial:
            self.serial.close()
        if self.bridge:
            self.bridge.close()

    def _read(self, count: int, deadline: float) -> bytes:
        result = bytearray()
        while len(result) < count and time.monotonic() < deadline:
            result.extend(self.serial.read(count - len(result)))
        if len(result) != count:
            raise LinkError(f"Serial timeout: expected {count} bytes, received {len(result)}")
        return bytes(result)

    def _send(self, frame: bytes):
        # Drain old bytes, but never accept an earlier response as this command's ACK.
        self.serial.reset_input_buffer()
        if self.serial.write(frame) != len(frame):
            raise LinkError("Incomplete serial write")
        self.serial.flush()

    def _modbus(self, function: int, address: int, values):
        frame = modbus_frame(self.device_id, function, address, values)
        self._send(frame)
        deadline = time.monotonic() + self.timeout
        prefix = self._read(2, deadline)
        if prefix[0] != self.device_id:
            raise LinkError("Modbus response device ID mismatch")
        if prefix[1] & 0x80:
            response = prefix + self._read(3, deadline)
        elif prefix[1] in (3, 4):
            count = self._read(1, deadline)
            if count[0] > 250:
                raise LinkError("Invalid Modbus byte count")
            response = prefix + count + self._read(count[0] + 2, deadline)
        else:
            response = prefix + self._read(6, deadline)
        if response[-2:] != crc16(response[:-2]):
            raise LinkError("Modbus CRC mismatch")
        if response[1] == function | 0x80:
            raise LinkError(f"Modbus exception 0x{response[2]:02x} at register 0x{address:04x}")
        if response[1] != function:
            raise LinkError("Modbus response function mismatch")
        if function in (3, 4):
            if response[2] != values * 2 or len(response) != 5 + values * 2:
                raise LinkError("Modbus response register count mismatch")
            return list(struct.unpack(">" + "H" * values, response[3:-2]))
        if response[:6] != frame[:6]:
            raise LinkError("Modbus write acknowledgement mismatch")
        return None

    def _custom(self, command: int, payload=b"", query=False):
        self._send(custom_frame(self.device_id, command, payload))
        if not query:
            # The official custom protocol sends motion without an ACK.
            return None
        deadline = time.monotonic() + self.timeout
        prefix = self._read(3, deadline)
        if prefix[:2] != b"\xa3\xb4" or not 2 <= prefix[2] <= 64:
            raise LinkError("Custom response header/length mismatch")
        frame = prefix + self._read(prefix[2] + 1, deadline)
        if frame[-1] != sum(frame[2:-1]) & 255:
            raise LinkError("Custom response checksum mismatch")
        if frame[3:5] != bytes([self.device_id, command]):
            raise LinkError("Custom response device ID/command mismatch")
        return frame[5:-1]

    def read_state(self):
        if self.protocol == "modbus":
            status, hardness, position, current = self._modbus(3, 0x0006, 4)
            mode = self._modbus(3, 0x000E, 1)[0]
            force = self._modbus(3, 0x0012, 1)[0]
            errors = self._modbus(3, 0x0023, 1)[0]
            fp_state = self._modbus(3, 0x002F, 1)[0] if mode == 1 else None
            if mode not in (0, 1) or fp_state not in (None, 0, 1, 2) or force > 100:
                raise LinkError("Invalid mode, force or force-position status feedback")
            holding = fp_state == 2 if mode == 1 else status == 2
            moving = fp_state == 0 if mode == 1 else status == 1
            slipping = status == 3 if mode == 0 else None
            labels = ("idle", "moving", "holding", "slipping")
            state_name = ("moving", "at_position", "holding")[fp_state] if mode == 1 else labels[status] if status < 4 else "unknown"
            if mode == 0 and self.adaptive_status_scheme == "three_is_holding":
                holding, moving, slipping = status == 3, status in (1, 2), None
                state_name = ("idle", "moving", "waiting_for_stable_grip", "holding")[status] if status < 4 else "unknown"
        else:
            data = self._custom(0x08, query=True)
            # SDK expects 7 bytes. The current manual also shows an extended
            # 11-byte payload: first 7 have the same layout; retain the tail raw.
            if len(data) not in (7, 11):
                raise LinkError("Unsupported custom grip-info payload length")
            status = data[0]
            hardness, position, current = struct.unpack("<HHH", data[1:7])
            mode = force = fp_state = None
            errors = None  # No polled error register in this protocol.
            holding, moving, slipping = status == 3, status in (1, 2), None
            state_name = ("idle", "moving", "waiting_for_stable_grip", "holding")[status] if status < 4 else "unknown"
        if status not in range(4) or not 0 <= position <= MEASURED_POSITION_MAX:
            raise LinkError("Invalid measured position/status feedback")
        return dict(protocol=self.protocol, device_id=self.device_id, status_raw=status,
                    adaptive_status_scheme=self.adaptive_status_scheme if self.protocol == "modbus" else "custom",
                    state=state_name, position_raw=position, hardness_raw=hardness,
                    current_raw=current, holding=holding, moving=moving, slipping=slipping,
                    control_mode=mode, target_force_percent=force, error_flags=errors,
                    force_position_state_raw=fp_state,
                    custom_extension_hex=data[7:].hex() if self.protocol == "custom" else "")

    def set_mode(self, mode):
        if self.protocol == "modbus":
            self._modbus(6, 0x000E, mode)
        else:
            self._custom(0x02, bytes([mode]))

    def set_force(self, force):
        if self.protocol == "modbus":
            self._modbus(6, 0x0012, force)
        else:
            self._custom(0x7A, struct.pack("<H", force))

    def adaptive(self, closing, speed):
        if self.protocol == "modbus":
            # Community hardware evidence: release via FC06 is ineffective on
            # some firmware; atomically write speed + action for BOTH directions.
            self._modbus(0x10, 0x000A, [speed, int(closing)])
        elif closing:
            self._custom(0x03, b"\0" + struct.pack("<H", speed))
        else:
            self._custom(0x01)

    def move_position(self, speed, position):
        if self.protocol == "modbus":
            self._modbus(0x10, 0x000F, [speed, position])
        else:
            self._custom(0x10, b"\x01" + struct.pack("<HH", speed, position) + b"\xff\xff")
