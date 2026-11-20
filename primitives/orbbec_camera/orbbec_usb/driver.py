"""Typed MCP snapshots and lifecycle-owned USB acquisition."""
import asyncio
import json
from pathlib import Path

from robonix_api import Primitive, Ok, Err
from orbbec_usb_mcp import (GetState_Request,GetState_Response,Snapshot_Request,Snapshot_Response,
                            CameraInfo_Request,CameraInfo_Response)
from .controller import Controller

ROOT = Path(__file__).resolve().parents[1]
provider = Primitive(id='robonix.primitive.orbbec.usb.camera',namespace='robonix/primitive/orbbec_camera',pkg_root=ROOT)
controller = None


async def invoke(response, method):
    try:
        if controller is None:raise RuntimeError('Camera provider not initialized')
        data = await asyncio.to_thread(getattr(controller,method))
        return response(ok=True,message='Live Orbbec SDK USB camera; no robot actions',
                        result_json=json.dumps(data,ensure_ascii=False,allow_nan=False))
    except Exception as error:
        return response(ok=False,message=str(error),result_json='{}')


@provider.mcp('robonix/primitive/orbbec_camera/get_state')
async def get_state(req:GetState_Request)->GetState_Response:
    """Read Orbbec USB camera connected/freshness, frame ID, serial, original resolution, intrinsics and calibration fingerprint. No robot or gripper commands. Does not return large image bytes."""
    return await invoke(GetState_Response,'state')


@provider.mcp('robonix/primitive/orbbec_camera/snapshot')
async def snapshot(req:Snapshot_Request)->Snapshot_Response:
    """Read latest fresh Orbbec color frame as original-resolution undistorted JPEG in result_json.jpeg_base64, with camera_key, intrinsics and frame ID. Consumers must validate against calibration. No HTTP camera service, no depth, no robot movement."""
    return await invoke(Snapshot_Response,'snapshot')


@provider.mcp('robonix/primitive/orbbec_camera/camera_info')
async def camera_info(req:CameraInfo_Request)->CameraInfo_Response:
    """Read live Orbbec color intrinsics K/D, serial, format, resolution and undistorted coordinate convention. No image bytes or robot movement. Depth is disabled for planar fixed-height grasp."""
    return await invoke(CameraInfo_Response,'camera_info')


@provider.on_init
def initialize(config):
    global controller
    try:
        controller = Controller(config,ROOT)
        return Ok()
    except Exception as error:return Err(str(error))


@provider.on_activate
def activate():
    try:
        if controller is None:raise RuntimeError('Initialize first')
        controller.start()
        return Ok()
    except Exception as error:return Err(str(error))


@provider.on_deactivate
def deactivate():
    if controller:controller.close()
    return Ok()


@provider.on_shutdown
def shutdown():
    global controller
    if controller:controller.close()
    controller = None
    return Ok()


if __name__ == '__main__':provider.run()
