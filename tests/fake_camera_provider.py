"""Production camera MCP/lifecycle with a deterministic private-pipe worker."""
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'primitives/orbbec_camera'))
from orbbec_usb import driver
from orbbec_usb.controller import Controller


def fixture(config,root):
    return Controller(config,root,command=[sys.executable,
        str(ROOT/'primitives/orbbec_camera/tests/fake_worker.py'),'--config-json',json.dumps(config)])


driver.Controller=fixture
if __name__=='__main__':driver.provider.run()
