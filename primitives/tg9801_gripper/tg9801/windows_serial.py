# SPDX-License-Identifier: MulanPSL-2.0
"""Expose a Windows COM port to one WSL process without a TCP listener.

The byte relay never constructs or replays Modbus commands. It owns the COM
handle only until close(), and uses a private PTY for the Linux serial client.
"""
from __future__ import annotations

import base64
import errno
import os
import platform
import pty
import re
import select
import shutil
import subprocess
import threading
import time
import tty


class WindowsSerialBridge:
    def __init__(self, port: str, baudrate: int = 115200):
        self._closed = False
        if not re.fullmatch(r"COM[1-9][0-9]*", port.upper()):
            raise ValueError("windows_port must be a COM number, e.g. COM4")
        executable = shutil.which("powershell.exe")
        if not executable:
            raise RuntimeError("Windows COM bridge requires WSL Windows interop/powershell.exe")
        # The relay runs in C# tasks, avoiding PowerShell runspace/thread issues.
        script = r'''
$ErrorActionPreference = 'Stop'
$ProgressPreference = 'SilentlyContinue'
[Console]::OutputEncoding = [System.Text.UTF8Encoding]::new($false)
Add-Type -TypeDefinition @'
using System;
using System.IO;
using System.IO.Ports;
using System.Threading.Tasks;
public static class RobonixSerialRelay {
 public static void Run(string name, int baud) {
  using (var serial = new SerialPort(name, baud, Parity.None, 8, StopBits.One)) {
   serial.ReadTimeout = 50; serial.WriteTimeout = 1000;
   serial.DtrEnable = false; serial.RtsEnable = false;
   serial.Open();
   var input = Console.OpenStandardInput(); var output = Console.OpenStandardOutput();
   Console.Error.WriteLine("RS485_READY");
   var tx = Task.Run(() => {
    var buf = new byte[4096]; int n;
    while ((n = input.Read(buf, 0, buf.Length)) > 0) serial.Write(buf, 0, n);
   });
   var rx = Task.Run(() => {
    var buf = new byte[4096];
    while (serial.IsOpen) {
     try { int n = serial.Read(buf, 0, buf.Length); output.Write(buf, 0, n); output.Flush(); }
     catch (TimeoutException) { }
    }
   });
   Task.WaitAny(tx, rx);
   serial.Close();
  }
 }
}
'@
try { [RobonixSerialRelay]::Run('PORT_NAME', BAUD_VALUE) }
catch {
 $failure = $_.Exception
 while ($failure.InnerException) { $failure = $failure.InnerException }
 [Console]::Error.WriteLine($failure.Message)
 exit 1
}
'''.replace("PORT_NAME", port.upper()).replace("BAUD_VALUE", str(int(baudrate)))
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)
        self.port = os.ttyname(self.slave)
        self._stop = threading.Event()
        self.error = ""
        command = [executable, "-NoLogo", "-NoProfile", "-NonInteractive", "-EncodedCommand",
                   base64.b64encode(script.encode("utf-16le")).decode("ascii")]
        pipes = dict(stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0)
        try:
            try:
                self.process = subprocess.Popen(command, **pipes)
            except OSError as error:
                if error.errno != errno.ENOEXEC or "microsoft" not in platform.release().lower():
                    raise
                # Missing WSLInterop binfmt handler; /init expects the path and original argv.
                self.process = subprocess.Popen(["/init", executable, *command], **pipes)
        except BaseException:
            os.close(self.master)
            os.close(self.slave)
            raise
        try:
            deadline = time.monotonic() + 15
            errors = []
            while time.monotonic() < deadline:
                ready, _, _ = select.select([self.process.stderr], [], [], .2)
                if ready:
                    line = self.process.stderr.readline().decode(errors="replace").strip()
                    if line == "RS485_READY":
                        break
                    errors.append(line)
                if self.process.poll() is not None:
                    raise RuntimeError(f"Could not open {port}: {' '.join(errors)}")
            else:
                raise RuntimeError(f"Windows COM relay startup timed out for {port}")
        except Exception:
            self.close()
            raise
        self.thread = threading.Thread(target=self._pump, daemon=True, name="hand-com-relay")
        self.thread.start()

    def _pump(self):
        try:
            while not self._stop.is_set():
                ready, _, _ = select.select([self.master, self.process.stdout], [], [], .1)
                for source in ready:
                    data = os.read(source if isinstance(source, int) else source.fileno(), 4096)
                    if not data:
                        raise OSError("Windows serial relay closed")
                    if source == self.master:
                        view = memoryview(data)
                        while view:
                            written = self.process.stdin.write(view)
                            if not written:
                                raise OSError("Windows serial relay write failed")
                            view = view[written:]
                    else:
                        view = memoryview(data)
                        while view:
                            view = view[os.write(self.master, view):]
        except (OSError, ValueError) as error:
            if not self._stop.is_set():
                self.error = str(error)

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._stop.set()
        if self.process.stdin:
            self.process.stdin.close()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        thread = getattr(self, "thread", None)
        if thread:
            thread.join(timeout=1)
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass
        for pipe in (self.process.stdout, self.process.stderr):
            if pipe:
                pipe.close()

    def __enter__(self):
        return self

    def __exit__(self, *_exc):
        self.close()
