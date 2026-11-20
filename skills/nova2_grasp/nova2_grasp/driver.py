"""Typed Robonix skill tools and lifecycle. Startup never moves hardware."""
from __future__ import annotations

import asyncio
import json
from pathlib import Path

from robonix_api import Err, Ok, Skill
from nova2_grasp_mcp import (
    Status_Request, Status_Response, DetectObject_Request, DetectObject_Response,
    PlanGrasp_Request, PlanGrasp_Response, Approach_Request, Approach_Response,
    Pick_Request, Pick_Response, Home_Request, Home_Response,
    PickAndPlace_Request, PickAndPlace_Response,
    Release_Request, Release_Response, Stop_Request, Stop_Response,
    TaskStatus_Request, TaskStatus_Response, SetJawAlignment_Request, SetJawAlignment_Response,
    CycleFromHover_Request, CycleFromHover_Response,
)

from .clients import Primitives
from .controller import Controller, DEFAULTS

ROOT = Path(__file__).resolve().parents[1]
provider = Skill(id="nova2_grasp", namespace="robonix/skill/nova2_grasp", pkg_root=ROOT)
controller = None


async def invoke(response_type, method, *args):
    try:
        if controller is None:
            raise RuntimeError("Nova2 grasp skill not initialized")
        result = await asyncio.to_thread(getattr(controller, method), *args)
        return response_type(ok=True, message="Read result_json. RUNNING means accepted, not completion; poll task_status.",
            result_json=json.dumps(result, ensure_ascii=False, allow_nan=False))
    except Exception as error:
        return response_type(ok=False, message=str(error), result_json="{}")


@provider.mcp("robonix/skill/nova2_grasp/status")
async def status(req: Status_Request) -> Status_Response:
    """Read Nova2 grasp calibration, saved ready pose, configured place_locations and release heights, jaw alignment, task status and arm/gripper feedback. No motion or gripper writes."""
    return await invoke(Status_Response, "status")


@provider.mcp("robonix/skill/nova2_grasp/detect_object")
async def detect_object(req: DetectObject_Request) -> DetectObject_Response:
    """Locate one target in the calibrated image. VLM supports a flat rectangular top or two opposite edges of a slender rounded target. YOLO-OBB uses rectangle geometry. Returns XY, closing direction, hover pose, taught grasp Z and annotated_image. Every target uses the same Z from calibration. No motion. Empty backend uses the configured default."""
    return await invoke(DetectObject_Response, "detect_object", req.object, req.backend, False)


@provider.mcp("robonix/skill/nova2_grasp/plan_grasp")
async def plan_grasp(req: PlanGrasp_Request) -> PlanGrasp_Response:
    """Detect the target and check IK for the hover, alignment and grasp poses. Every target uses the taught grasp Z from calibration. Report the poses and annotated_image. No movement or gripper writes. IK checks endpoints only."""
    return await invoke(PlanGrasp_Response, "detect_object", req.object, req.backend, True)


@provider.mcp("robonix/skill/nova2_grasp/approach")
async def approach(req: Approach_Request) -> Approach_Response:
    """Move to the saved ready TCP pose if needed, then detect and align vertically above the target. Stops at hover. Supply object or fresh plan_id. After any move to ready, the skill captures a new image and replans. Returns RUNNING. Poll task_status."""
    return await invoke(Approach_Response, "start", "approach", req.object, req.backend, req.plan_id)


@provider.mcp("robonix/skill/nova2_grasp/pick")
async def pick(req: Pick_Request) -> Pick_Response:
    """Real grasp: move to ready if needed, detect, align, descend, grip, and lift vertically. Stays above the pickup point holding the object. Requires confirmed jaw alignment, enabled idle arm and empty gripper. Supply object or fresh plan_id. Poll task_status to completion."""
    return await invoke(Pick_Response, "start", "pick", req.object, req.backend, req.plan_id)


@provider.mcp("robonix/skill/nova2_grasp/pick_and_place")
async def pick_and_place(req: PickAndPlace_Request) -> PickAndPlace_Response:
    """Real motion and gripper: pick the object, carry it to a configured destination and release there at its configured height. Set destination to the exact name from status.place_locations, e.g. 白盒子. Supply object or fresh plan_id, with the unused field empty. Empty backend selects the default. Requires confirmed jaw alignment, enabled idle arm and empty gripper. Poll task_status to completion."""
    return await invoke(PickAndPlace_Response, "start", "pick_and_place", req.object, req.backend,
                        req.plan_id, req.destination)


@provider.mcp("robonix/skill/nova2_grasp/home")
async def home(req: Home_Request) -> Home_Response:
    """REAL MOTION: return to the saved ready/return joint pose. If currently low and vertical, lift to configured hover Z first. Does not open or close the gripper; preserves held objects. Returns RUNNING; poll task_status."""
    return await invoke(Home_Response, "start", "home")


@provider.mcp("robonix/skill/nova2_grasp/cycle_from_hover")
async def cycle_from_hover(req: CycleFromHover_Request) -> CycleFromHover_Response:
    """REAL MOTION + GRIPPER: from the CURRENT manually verified vertical hover, open -> descend to calibrated Z -> grip/confirm holding -> lift -> descend to SAME original position -> release -> lift -> return saved ready joints. Requires confirmed jaw offset, empty idle gripper, enabled idle arm and hover Z within 2mm. Does NOT re-detect while the gripper occludes the object. Preserves current XY and yaw. Returns RUNNING; poll task_status. Use after operator checks hover alignment."""
    return await invoke(CycleFromHover_Response, "start", "cycle_from_hover")


@provider.mcp("robonix/skill/nova2_grasp/release")
async def release(req: Release_Request) -> Release_Response:
    """Open/release TG9801 at CURRENT location, may drop a held object. NO arm motion. Returns RUNNING; poll task_status for measured not-moving and not-holding feedback; full mechanical opening is not calibrated."""
    return await invoke(Release_Response, "start", "release")


@provider.mcp("robonix/skill/nova2_grasp/stop")
async def stop(req: Stop_Request) -> Stop_Response:
    """Cancel the grasp task and stop Nova2 immediately; prevents subsequent moves/closure. No automatic return or jaw opening. TG9801 has no verified stop command, so an already-issued jaw action may finish. Use this tool to stop a multi-step grasp, not only the arm primitive stop."""
    return await invoke(Stop_Response, "stop")


@provider.mcp("robonix/skill/nova2_grasp/task_status")
async def task_status(req: TaskStatus_Request) -> TaskStatus_Response:
    """Poll latest asynchronous grasp/home/approach/release task. Only SUCCEEDED confirms all required measured steps; inspect result.gripper and actual object retention. RUNNING is not success; failure never automatically retries, opens, or returns."""
    return await invoke(TaskStatus_Response, "task_status")


@provider.mcp("robonix/skill/nova2_grasp/set_jaw_alignment")
async def set_jaw_alignment(req: SetJawAlignment_Request) -> SetJawAlignment_Response:
    """Configuration only, NO motion: set two-finger closing-axis offset relative to Tool0 +X in degrees (-180..180). X=0, Y=90; confirm=true only after observing hover alignment. Persists to runtime.json, invalidates old plans, affects next detection/pick. Unconfirmed alignment permits approach but blocks descent/pick."""
    return await invoke(SetJawAlignment_Response, "set_jaw_alignment", req.offset_deg, req.confirmed)


@provider.on_init
def initialize(config):
    global controller
    try:
        controller = Controller(config, ROOT, Primitives(provider, {**DEFAULTS, **config}))
        return Ok()
    except Exception as error:
        return Err(str(error))


@provider.on_activate
def activate():
    # Perception and primitive channels are opened only on explicit tool calls.
    return Ok() if controller is not None else Err("Initialize first")


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
