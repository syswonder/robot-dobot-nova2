"""Typed MCP tools, gRPC lifecycle and no arm/ROS dependency."""
from __future__ import annotations

import asyncio
import json
from pathlib import Path

from robonix_api import Err, Ok, Primitive
from tg9801_mcp import (
    Info_Request, Info_Response, GetState_Request, GetState_Response,
    Open_Request, Open_Response, Close_Request, Close_Response,
    Grip_Request, Grip_Response, SetParameters_Request, SetParameters_Response,
    MovePosition_Request, MovePosition_Response, WaitForState_Request, WaitForState_Response,
)

from .controller import Controller

provider = Primitive(id="robonix.primitive.hkv.tg9801.gripper", namespace="robonix/primitive/tg9801",
                     pkg_root=Path(__file__).resolve().parent.parent)
controller: Controller | None = None


async def invoke(response_type, method, *args):
    try:
        if controller is None:
            raise RuntimeError("TG9801 provider not initialized")
        result = await asyncio.to_thread(getattr(controller, method), *args)
        return response_type(ok=True, message="See result_json; command transmission is not completion",
                             result_json=json.dumps(result, ensure_ascii=False, allow_nan=False))
    except Exception as error:
        return response_type(ok=False, message=str(error), result_json="{}")


@provider.mcp("robonix/primitive/tg9801/info")
async def info(req: Info_Request) -> Info_Response:
    """Read TG9801 gripper capabilities, serial protocol and runtime defaults. Force percent is a device setting, NOT newtons. No stop command or calibrated millimetre width is available."""
    return await invoke(Info_Response, "info")


@provider.mcp("robonix/primitive/tg9801/get_state")
async def get_state(req: GetState_Request) -> GetState_Response:
    """Read measured TG9801 position_raw 0..1000, moving, holding, protocol-specific status and available errors. Raw position direction is uncalibrated. Only holding=true reports device-confirmed stable grip; no arm motion."""
    return await invoke(GetState_Response, "state")


@provider.mcp("robonix/primitive/tg9801/open")
async def open_gripper(req: Open_Request) -> Open_Response:
    """Release/open TG9801 only. speed=0 uses runtime default; otherwise raw 200..configured max. Modbus applies speed; custom release has no speed field. May drop a held object. Query state afterward; no arm movement and no safety stop."""
    return await invoke(Open_Response, "command", "open", req.speed)


@provider.mcp("robonix/primitive/tg9801/grip")
async def grip(req: Grip_Request) -> Grip_Response:
    """Adaptive grasp/close TG9801 only. speed=0 and force_percent=0 use defaults; speed raw 200..1500 and force device percent 1..configured ceiling, NOT N. Poll wait_for_state holding or get_state to verify grasp. Does not move the arm."""
    return await invoke(Grip_Response, "command", "grip", req.speed, req.force_percent)


@provider.mcp("robonix/primitive/tg9801/close")
async def close_gripper(req: Close_Request) -> Close_Response:
    """Close TG9801 using adaptive grasp, equivalent to grip. speed=0/force_percent=0 use defaults. A close command does not prove an object is held; read measured state. Never move Nova2 for a gripper-only request."""
    return await invoke(Close_Response, "command", "close", req.speed, req.force_percent)


@provider.mcp("robonix/primitive/tg9801/set_parameters")
async def set_parameters(req: SetParameters_Request) -> SetParameters_Response:
    """Set TG9801 provider runtime defaults for NEXT commands only; does not write hardware or alter current holding force. Zero keeps current default. speed raw 200..max, force_percent 1..max, NOT N. Restart restores manifest defaults."""
    return await invoke(SetParameters_Response, "parameters", req.speed, req.force_percent)


@provider.mcp("robonix/primitive/tg9801/move_position")
async def move_position(req: MovePosition_Request) -> MovePosition_Response:
    """Optional TG9801 force-position mode target in RAW 0..1000, NOT mm or normalized width. Disabled until position_control_enabled=true after manual endpoint/direction checks. speed/force zero use defaults. Query measured state; no arm motion."""
    return await invoke(MovePosition_Response, "command", "move_position", req.speed,
                        req.force_percent, req.position_raw)


@provider.mcp("robonix/primitive/tg9801/wait_for_state")
async def wait_for_state(req: WaitForState_Request) -> WaitForState_Response:
    """Poll TG9801 only, requiring 3 matching measured samples. condition=holding, idle, or not_moving; timeout_s=0 selects 5s, maximum 10s. Timeout does NOT stop or reverse jaws. idle/not_moving never proves full opening or successful grasp."""
    return await invoke(WaitForState_Response, "wait_for_state", req.condition, req.timeout_s or 5)


@provider.on_init
def initialize(config):
    global controller
    try:
        controller = Controller(config)
        return Ok()
    except Exception as error:
        return Err(str(error))


@provider.on_activate
def activate():
    try:
        if controller is None:
            raise RuntimeError("Initialize TG9801 first")
        controller.start()
        return Ok()
    except Exception as error:
        if controller:
            controller.close()
        return Err(str(error))


@provider.on_deactivate
def deactivate():
    if controller:
        controller.close()
    return Ok()


@provider.on_shutdown
def shutdown():
    global controller
    if controller:
        controller.close()
        controller = None
    return Ok()


if __name__ == "__main__":
    provider.run()
