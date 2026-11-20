"""Dobot Nova2 typed MCP capabilities and Robonix lifecycle."""
from __future__ import annotations

import asyncio
import json
from pathlib import Path

from robonix_api import Err, Ok, Primitive

from .controller import Controller
from .protocol import ControllerError
from nova2_mcp import (
    GetState_Request, GetState_Response,
    Enable_Request, Enable_Response,
    Disable_Request, Disable_Response,
    ClearError_Request, ClearError_Response,
    Stop_Request, Stop_Response,
    MoveJoints_Request, MoveJoints_Response,
    MovePose_Request, MovePose_Response,
    MoveQuaternion_Request, MoveQuaternion_Response,
    MotionStatus_Request, MotionStatus_Response,
    ForwardKinematics_Request, ForwardKinematics_Response,
    InverseKinematics_Request, InverseKinematics_Response,
)

provider = Primitive(id="nova2_arm", namespace="robonix/primitive/dobot_nova2",
                     pkg_root=Path(__file__).resolve().parent.parent)
controller: Controller | None = None


async def invoke(response_type, method, *args):
    try:
        if controller is None:
            raise ControllerError("Nova2 provider is not active")
        result = await asyncio.to_thread(getattr(controller, method), *args)
        return response_type(ok=True, message="See result_json for measured state or motion status",
                             result_json=json.dumps(result, ensure_ascii=False, allow_nan=False))
    except (ControllerError, ValueError, TypeError) as error:
        return response_type(ok=False, message=str(error), result_json="{}")


@provider.mcp("robonix/primitive/dobot_nova2/get_state")
async def get_state(req: GetState_Request) -> GetState_Response:
    """Read live robot mode, six joint angles in degrees, TCP pose in mm/degrees, quaternion xyzw, alarms and current motion. Pose uses User0/Tool0."""
    return await invoke(GetState_Response, "state")


@provider.mcp("robonix/primitive/dobot_nova2/enable")
async def enable(req: Enable_Request) -> Enable_Response:
    """Enable the idle Nova2 using the controller's existing payload settings. Does not move, clear alarms or power on."""
    return await invoke(Enable_Response, "enable")


@provider.mcp("robonix/primitive/dobot_nova2/disable")
async def disable(req: Disable_Request) -> Disable_Response:
    """Stop any running motion and disable the Nova2; verify disabled feedback."""
    return await invoke(Disable_Response, "disable")


@provider.mcp("robonix/primitive/dobot_nova2/clear_error")
async def clear_error(req: ClearError_Request) -> ClearError_Response:
    """Request clearing controller alarms after their cause is resolved. Does not enable or resume."""
    return await invoke(ClearError_Response, "clear_error")


@provider.mcp("robonix/primitive/dobot_nova2/stop")
async def stop(req: Stop_Request) -> Stop_Response:
    """Stop the controller motion queue and confirm idle feedback. Use immediately when the user requests stopping."""
    return await invoke(Stop_Response, "stop")


@provider.mcp("robonix/primitive/dobot_nova2/move_joints")
async def move_joints(req: MoveJoints_Request) -> MoveJoints_Response:
    """Move to six absolute joint angles J1..J6 in DEGREES using V3 JointMovJ (V4 MovJ joint mode). velocity_percent=0 selects default. Returns an accepted motion run_id, not arrival. Poll motion_status until terminal."""
    return await invoke(MoveJoints_Response, "move_joints", req.joints_deg, req.velocity_percent)


@provider.mcp("robonix/primitive/dobot_nova2/move_pose")
async def move_pose(req: MovePose_Request) -> MovePose_Response:
    """Move to absolute [X,Y,Z,Rx,Ry,Rz] in MILLIMETRES and DEGREES, User0/Tool0. motion must be MovJ or MovL; velocity_percent=0 selects default for either. IK checked first; collision-free path is not planned. Poll motion_status."""
    return await invoke(MovePose_Response, "move_pose", req.pose_mm_deg, req.motion, req.velocity_percent)


@provider.mcp("robonix/primitive/dobot_nova2/move_quaternion")
async def move_quaternion(req: MoveQuaternion_Request) -> MoveQuaternion_Response:
    """Move to absolute XYZ in MILLIMETRES plus normalized quaternion [qx,qy,qz,qw] in User0/Tool0. Converts xyz extrinsic Euler (Rz*Ry*Rx) to native Rx/Ry/Rz; rejects Euler singularities. Same motion/speed rules as move_pose. Poll motion_status."""
    return await invoke(MoveQuaternion_Response, "move_quaternion", req.position_mm, req.quaternion_xyzw, req.motion, req.velocity_percent)


@provider.mcp("robonix/primitive/dobot_nova2/motion_status")
async def motion_status(req: MotionStatus_Request) -> MotionStatus_Response:
    """Read motion status by run_id (empty=latest), optionally wait_s 0..10. Only SUCCEEDED means arrival verified. FAILED, CANCELED, UNKNOWN and NOT_FOUND are not success. Continue polling active RUNNING/PENDING tasks before issuing another move."""
    return await invoke(MotionStatus_Response, "motion_status", req.run_id, req.wait_s)


@provider.mcp("robonix/primitive/dobot_nova2/forward_kinematics")
async def forward_kinematics(req: ForwardKinematics_Request) -> ForwardKinematics_Response:
    """Compute controller forward kinematics for six joint angles in degrees, returning User0/Tool0 pose in mm/degrees. Does not move."""
    return await invoke(ForwardKinematics_Response, "forward_kinematics", req.joints_deg)


@provider.mcp("robonix/primitive/dobot_nova2/inverse_kinematics")
async def inverse_kinematics(req: InverseKinematics_Request) -> InverseKinematics_Response:
    """Compute controller inverse kinematics for [X,Y,Z,Rx,Ry,Rz] in mm/degrees, User0/Tool0, selecting the solution nearest the actual current joints. Does not move or prove path clearance."""
    return await invoke(InverseKinematics_Response, "inverse_kinematics", req.pose_mm_deg)


@provider.on_init
def initialize(config):
    global controller
    try:
        controller = Controller(config)
        return Ok()
    except (ValueError, TypeError, OSError) as error:
        return Err(str(error))


@provider.on_activate
def activate():
    try:
        if controller is None:
            raise ControllerError("Initialize the provider first")
        controller.start()
        return Ok()
    except (ControllerError, OSError) as error:
        if controller is not None:
            controller.close()
        return Err(str(error))


@provider.on_deactivate
def deactivate():
    global controller
    if controller is not None:
        config = controller.config
        controller.close()
        controller = Controller(config)
    return Ok()


@provider.on_shutdown
def shutdown():
    global controller
    if controller is not None:
        controller.close()
        controller = None
    return Ok()


if __name__ == "__main__":
    provider.run()
