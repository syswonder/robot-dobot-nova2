"""PTY firmware emulator. Never opens COM or arm sockets."""
from __future__ import annotations

import os
import pty
import select
import struct
import threading
import tty


def checksum_crc(data):
    # Independent CRC implementation for the device side.
    value = 65535
    for byte in data:
        value ^= byte
        for _ in range(8):
            odd = value % 2
            value //= 2
            if odd:
                value ^= 40961
    return struct.pack("<H", value)


class FakeGripper:
    def __init__(self, protocol="modbus", extended=False, adaptive_holding_status=2):
        self.protocol = protocol
        self.adaptive_holding_status = adaptive_holding_status
        self.extended = extended
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.config = dict(port=os.ttyname(self.slave), windows_port="", baudrate=115200,
                           protocol=protocol, timeout_s=.1)
        self.regs = {6: 0, 7: 0, 8: 500, 9: 48, 14: 0, 18: 20, 35: 0, 47: 1}
        self.writes = []
        self.requests = []
        self.drop_ack = False
        self.bad_crc = False
        self.wrong_id = False
        self.wrong_command = False
        self.exception_address = None
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=1)
        os.close(self.master)
        os.close(self.slave)

    def run(self):
        buf = bytearray()
        while not self.stop.is_set():
            if not select.select([self.master], [], [], .02)[0]:
                continue
            try:
                buf.extend(os.read(self.master, 4096))
            except OSError:
                return
            while buf:
                if buf[0] == 0xA3:
                    if len(buf) < 3:
                        break
                    length = buf[2] + 4
                elif len(buf) >= 7 and buf[1] == 0x10:
                    length = 9 + buf[6]
                elif len(buf) >= 2 and buf[1] in (3, 4, 6):
                    length = 8
                else:
                    if len(buf) < 7:
                        break
                    del buf[0]
                    continue
                if len(buf) < length:
                    break
                request = bytes(buf[:length]);del buf[:length]
                self.requests.append(request)
                response = self.custom(request) if request[:2] == b"\xa3\xb4" else self.modbus(request)
                if response:
                    # Fragment packets to exercise complete-frame assembly.
                    os.write(self.master, response[:2]);os.write(self.master, response[2:])

    def modbus(self, request):
        if self.protocol != "modbus" or request[-2:] != checksum_crc(request[:-2]):
            return None
        ident, function, addr, count = struct.unpack(">BBHH", request[:6])
        if function == 3:
            if addr == self.exception_address:
                body = bytes([ident, 0x83, 2])
            else:
                values = [self.regs.get(addr+i, 0) for i in range(count)]
                body = bytes([ident, 3, count*2]) + struct.pack(">"+"H"*count, *values)
        else:
            values = [count] if function == 6 else list(struct.unpack(">"+"H"*count, request[7:-2]))
            self.writes.append((addr, values))
            for i, value in enumerate(values):
                self.regs[addr+i] = value
            if addr == 10:
                closing = bool(values[1]);self.regs[6] = self.adaptive_holding_status if closing else 0
                self.regs[8] = 0 if closing else 1000
            if addr == 15:
                self.regs[8] = values[1];self.regs[47] = 1
            if self.drop_ack:
                return None
            body = request[:6]
        if self.wrong_id:
            body = b"\x02" + body[1:]
        result = body + checksum_crc(body)
        return result[:-1] + bytes([result[-1] ^ 1]) if self.bad_crc else result

    def custom(self, request):
        if self.protocol != "custom" or sum(request[2:-1]) & 255 != request[-1]:
            return None
        ident, cmd = request[3:5]
        if cmd == 8:
            data = bytes([self.regs[6]]) + struct.pack("<HHH", self.regs[7], self.regs[8], self.regs[9])
            if self.extended:
                data += b"\x37\0\0\0"
            body = bytes([len(data)+2, 2 if self.wrong_id else ident, 9 if self.wrong_command else cmd]) + data
            result = b"\xa3\xb4" + body + bytes([sum(body) & 255])
            return result[:-1] + bytes([result[-1] ^ 1]) if self.bad_crc else result
        data = request[5:-1]
        self.writes.append((cmd, list(data)))
        if cmd in (1, 3):
            self.regs[6] = 3 if cmd == 3 else 0
            self.regs[8] = 0 if cmd == 3 else 1000
        return None
