"""Measured grasp sequencing; all physical I/O goes through primitives."""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import threading
import time
import uuid

import numpy as np
from scipy.spatial.transform import Rotation

from .geometry import finite_vector, side_grasp_geometry, target_geometry, vertical_tilt
from .vision import Camera, VlmDetector, YoloDetector, validate_detection, save_overlay


DEFAULTS = dict(
    calibration_path="../../config/grasp/calibration.json", ready_pose_path="../../config/grasp/ready_pose.json",
    place_locations_path="",
    runtime_path="../../config/grasp/runtime.json", artifacts_dir="../../.artifacts/grasp",
    camera_provider_id="orbbec_camera", camera_max_frame_age_s=3.,
    backend="vlm", approach_height_mm=200., velocity_percent=20, gripper_speed=300, force_percent=20,
    vertical_tolerance_deg=2., jaw_axis_offset_deg=0., jaw_alignment_confirmed=False,
    min_confidence=.6, plan_ttl_s=120., motion_timeout_s=305., gripper_timeout_s=8.,
    max_calibration_error_mm=20., square_ratio=1.12, arm_provider_id="nova2_arm",
    gripper_provider_id="tg9801_gripper", vlm_base_url="", vlm_api_key="", vlm_model="",
    vlm_timeout_s=45., yolo_model_path="", yolo_class_aliases_json="{}",
)


class Controller:
    def __init__(self, config, root, primitives, camera=None, detectors=None):
        self.root = Path(root)
        self.config = {**DEFAULTS, **config}
        c = self.config
        for name, lo, hi in [("approach_height_mm", 20, 400), ("vertical_tolerance_deg", .01, 15),
                              ("velocity_percent", 1, 80), ("gripper_speed", 200, 1500),
                              ("force_percent", 1, 50), ("min_confidence", 0, 1),
                              ("plan_ttl_s", 10, 300), ("motion_timeout_s", 10, 600),
                              ("gripper_timeout_s", 1, 10), ("vlm_timeout_s", 1, 120),
                              ("max_calibration_error_mm", .1, 20), ("square_ratio", 1, 1.5),
                              ("camera_max_frame_age_s", .1, 10)]:
            value = c[name]
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lo <= value <= hi:
                raise ValueError(f"{name} must be in {lo}..{hi}")
        for name in ("velocity_percent", "gripper_speed", "force_percent"):
            if not isinstance(c[name], int):
                raise ValueError(name+" must be an integer")
        self.calibration_path = self.path(c["calibration_path"])
        self.ready_path = self.path(c["ready_pose_path"])
        self.runtime_path = self.path(c["runtime_path"])
        self.artifacts = self.path(c["artifacts_dir"])
        self.artifacts.mkdir(parents=True, exist_ok=True)
        self.calibration = json.loads(self.calibration_path.read_text())
        self.calibration_digest = self.digest(self.calibration_path)
        cal = self.calibration
        if cal.get("format") != "nova2-planar-handeye-v1" or not cal.get("accepted"):
            raise ValueError("必须使用已通过检查的平面标定 JSON")
        if cal.get("frames") != dict(user=0, tool=0):
            raise ValueError("标定必须使用 User0/Tool0")
        if not math.isfinite(cal["max_error_mm"]) or cal["max_error_mm"] > c["max_calibration_error_mm"]:
            raise ValueError("标定最大拟合误差超过本技能配置上限")
        finite_vector(cal["orientation_rxryrz_deg"], 3, "calibration orientation")
        if not math.isfinite(cal["z_mm"]):
            raise ValueError("无效抓取高度")
        self.ready = json.loads(self.ready_path.read_text())
        finite_vector(self.ready["pose_mm_deg"], 6, "ready pose")
        finite_vector(self.ready["joints_deg"], 6, "ready joints")
        if self.ready.get("user") != 0 or self.ready.get("tool") != 0:
            raise ValueError("准备位必须使用 User0/Tool0")
        self.ready_digest = self.digest(self.ready_path)
        if self.ready["pose_mm_deg"][2] < cal["z_mm"]+c["approach_height_mm"]:
            raise ValueError("保存的准备位低于抓取上方高度，请修改接近距离或重新示教准备位")
        self.place_locations_path = self.path(c["place_locations_path"]) if c["place_locations_path"] else None
        self.place_locations = {}
        if self.place_locations_path is not None:
            data = self.place_locations_path.read_bytes()
            self.place_locations_digest = hashlib.sha256(data).hexdigest()
            self.place_locations = self._load_place_locations(json.loads(data))
        self.alignment = dict(jaw_axis_offset_deg=c["jaw_axis_offset_deg"], confirmed=c["jaw_alignment_confirmed"])
        if self.runtime_path.exists():
            self.alignment = json.loads(self.runtime_path.read_text())
        self._validate_alignment(self.alignment["jaw_axis_offset_deg"], self.alignment["confirmed"])
        self.primitives = primitives
        self.camera = camera or Camera(c,primitives)
        self.detectors = detectors or dict(vlm=VlmDetector(c), yolo=YoloDetector(c))
        self.lock = threading.RLock()
        self.issue_lock = threading.Lock()
        self.compute_lock = threading.Lock()
        self.cancel = threading.Event()
        self.thread = None
        self.plans = {}
        self.state = dict(state="IDLE", message="未执行抓取", run_id="", stage="", result=None)

    def path(self, value):
        path = Path(value).expanduser()
        return path if path.is_absolute() else (self.root/path).resolve()

    @staticmethod
    def digest(path):
        return hashlib.sha256(path.read_bytes()).hexdigest()

    def unchanged(self):
        if self.digest(self.calibration_path) != self.calibration_digest or self.digest(self.ready_path) != self.ready_digest:
            raise ValueError("标定/准备位文件已变化，请重启本技能，禁止混用旧计划")
        if self.place_locations_path is not None and self.digest(self.place_locations_path) != self.place_locations_digest:
            raise ValueError("放置地点文件已变化，请重启本技能")

    def _load_place_locations(self, locations):
        if not isinstance(locations, dict):
            raise ValueError("放置地点文件必须是以地点名称为键的 JSON 对象")
        result = {}
        for name, location in locations.items():
            if not name or name != name.strip():
                raise ValueError("放置地点名称不能为空或带有首尾空格")
            if not isinstance(location, dict) or location.keys() != {"xy_mm", "release_height_mm"}:
                raise ValueError(f"放置地点 {name} 必须包含 xy_mm 和 release_height_mm")
            xy = location["xy_mm"]
            if not isinstance(xy, list) or len(xy) != 2 or any(
                type(value) not in (int, float) or not math.isfinite(value) for value in xy
            ):
                raise ValueError(f"放置地点 {name} 的 xy_mm 必须是两个有限数值")
            height = location["release_height_mm"]
            if type(height) not in (int, float) or not math.isfinite(height) or height <= 0:
                raise ValueError(f"放置地点 {name} 的 release_height_mm 必须是正数")
            release_z = self.calibration["z_mm"] + height
            if not math.isfinite(release_z):
                raise ValueError(f"放置地点 {name} 的释放 Z 无效")
            result[name] = dict(xy_mm=xy, release_height_mm=height, release_z_mm=release_z)
        return result

    def active(self):
        return self.thread is not None and self.thread.is_alive()

    def task_status(self):
        with self.lock:
            return copy.deepcopy(self.state)

    def status(self):
        self.unchanged()
        result = dict(task=self.task_status(), backend=self.config["backend"],
            approach_height_mm=self.config["approach_height_mm"], grasp_z_mm=self.calibration["z_mm"],
            hover_z_mm=self.calibration["z_mm"]+self.config["approach_height_mm"],
            calibration_samples=self.calibration["samples"], calibration_max_error_mm=self.calibration["max_error_mm"],
            ready=self.ready, alignment=self.alignment.copy(), calibration_path=str(self.calibration_path),
            ready_pose_path=str(self.ready_path),velocity_percent=self.config["velocity_percent"],
            place_locations_path=str(self.place_locations_path or ""),
            place_locations=copy.deepcopy(self.place_locations),
            camera_provider_id=self.config["camera_provider_id"],camera_source="orbbec_sdk_usb",
            yolo_weights_configured=bool(self.config["yolo_model_path"]))
        for group in ("arm", "gripper", "camera"):
            try:
                result[group] = self.primitives.call(group, "get_state")
            except Exception as error:
                result[group] = dict(available=False, error=str(error))
        return result

    @staticmethod
    def _validate_alignment(offset, confirmed):
        if isinstance(offset, bool) or not isinstance(offset, (int, float)) or not math.isfinite(offset) or not -180 <= offset <= 180:
            raise ValueError("jaw_axis_offset_deg must be -180..180 degrees")
        if not isinstance(confirmed, bool):
            raise ValueError("confirmed must be boolean")

    def set_jaw_alignment(self, offset_deg, confirmed):
        self._validate_alignment(offset_deg, confirmed)
        with self.lock:
            if self.active() or self.compute_lock.locked():
                raise ValueError("任务或识别规划正在执行，不能修改夹爪方向")
            self.alignment = dict(jaw_axis_offset_deg=float(offset_deg), confirmed=confirmed)
            self.runtime_path.parent.mkdir(parents=True, exist_ok=True)
            temp = self.runtime_path.with_suffix(".tmp")
            temp.write_text(json.dumps(self.alignment, indent=2)+"\n")
            temp.replace(self.runtime_path)
            self.plans.clear()
            return dict(**self.alignment, physical_motion=False, applies="next plan")

    def detect_object(self, description, backend="", check_ik=False, internal=False):
        if not description or len(description) > 500:
            raise ValueError("请给出 1..500 字符的目标描述")
        backend = backend or self.config["backend"]
        if backend not in ("vlm", "yolo"):
            raise ValueError("backend must be vlm or yolo (empty uses configured default)")
        with self.lock:
            if self.active() and not internal:
                raise ValueError("抓取任务执行中，请先等待结束或停止")
            if not self.compute_lock.acquire(blocking=False):
                raise ValueError("已有识别/规划请求正在执行")
        try:
            self.unchanged()
            frame = self.camera.capture(self.calibration)
            ident = uuid.uuid4().hex
            (self.artifacts/(ident+"-source.jpg")).write_bytes(frame["jpeg"])
            raw = self.detectors[backend].detect(frame, description)
            (self.artifacts/(ident+"-detection.json")).write_text(json.dumps(raw, ensure_ascii=False, indent=2)+"\n")
            detection = validate_detection(raw, frame, self.config["min_confidence"])
            ref_yaw = self.calibration["orientation_rxryrz_deg"][2]
            offset = self.alignment["jaw_axis_offset_deg"]
            if detection["grasp_geometry"] == "side_pair":
                geometry = side_grasp_geometry(self.calibration, np.asarray(detection["side_points_px"]),
                                               offset, ref_yaw)
            else:
                geometry = target_geometry(self.calibration, detection["top_corners_px"],
                    frame["image"].shape[1], frame["image"].shape[0], offset,
                    ref_yaw, self.config["square_ratio"])
                xy = np.asarray(geometry["xy_mm"])
                axis = np.asarray(geometry["closing_axis_xy"])
                inv = np.linalg.inv(np.asarray(self.calibration["pixel_to_xy"]))
                endpoints = []
                for point in (xy-axis*25, xy+axis*25):
                    p = inv @ [*point, 1.]
                    endpoints.append((p[:2]/p[2]).tolist())
                geometry["closing_axis_pixels"] = endpoints
            z = self.calibration["z_mm"]
            hover_z = z+self.config["approach_height_mm"]
            # Exact vertical orientation; ready pose may have a different tilt.
            nominal_yaw = ref_yaw
            hover = [*geometry["xy_mm"], hover_z, 180., 0., geometry["yaw_deg"]]
            unaligned = [*hover[:3], 180., 0., nominal_yaw]
            grasp = [*hover[:2], z, *hover[3:]]
            overlay_path = self.artifacts/(ident+"-detection.jpg")
            save_overlay(frame, detection, geometry, overlay_path)
            plan = dict(plan_id=ident, object=description, backend=backend, created_at=time.time(),
                expires_at=frame["captured_at"]+self.config["plan_ttl_s"],
                calibration_digest=self.calibration_digest, camera_key=frame["camera_key"],
                frame_id=frame["frame_id"], detection=detection, geometry=geometry,
                unaligned_hover_pose_mm_deg=unaligned, hover_pose_mm_deg=hover, grasp_pose_mm_deg=grasp,
                grasp_height_source="calibration", hover_reference_z_mm=z,
                approach_height_mm=self.config["approach_height_mm"], ready=self.ready,
                calibration_max_error_mm=self.calibration["max_error_mm"],
                jaw_alignment_confirmed=self.alignment["confirmed"], alignment=self.alignment.copy(),
                annotated_image=str(overlay_path), ik_checked=False, physical_motion=False)
            if check_ik:
                self.check_plan(plan)
            with self.lock:
                self.plans = {k:v for k,v in self.plans.items() if v["expires_at"] > time.time()}
                self.plans[ident] = plan
            (self.artifacts/(ident+"-plan.json")).write_text(json.dumps(plan, ensure_ascii=False, indent=2)+"\n")
            return copy.deepcopy(plan)
        finally:
            self.compute_lock.release()

    def check_plan(self, plan):
        self.unchanged()
        if plan["expires_at"] <= time.time():
            raise ValueError("视觉计划已过期，请重新识别；不使用旧目标")
        if plan["alignment"] != self.alignment:
            raise ValueError("夹爪方向配置已变化，请重新识别")
        frame = self.camera.capture(self.calibration)
        if frame["camera_key"] != plan["camera_key"]:
            raise ValueError("相机模式已变化，请重新标定或恢复相机模式")
        solutions = {}
        for name in ("unaligned_hover_pose_mm_deg", "hover_pose_mm_deg", "grasp_pose_mm_deg"):
            solutions[name] = self.primitives.call("arm", "inverse_kinematics", pose_mm_deg=plan[name])
        plan.update(ik_checked=True, ik_solutions=solutions)
        return plan

    def start(self, action, description="", backend="", plan_id="", destination=""):
        if action not in ("pick", "pick_and_place", "approach", "home", "release", "cycle_from_hover"):
            raise ValueError("不支持的抓取动作")
        destination = destination.strip()
        if action == "pick_and_place" and destination not in self.place_locations:
            raise ValueError(f"未知放置地点：{destination or '未指定'}。已配置：{', '.join(self.place_locations) or '无'}")
        if action != "pick_and_place" and destination:
            raise ValueError("仅 pick_and_place 接受放置地点")
        with self.lock:
            if self.active() or self.compute_lock.locked():
                raise ValueError("已有任务或识别规划正在执行")
            self.unchanged()
            if action == "cycle_from_hover" and not self.alignment["confirmed"]:
                raise ValueError("请先核对并确认夹爪闭合方向")
            plan = None
            if action in ("pick", "pick_and_place", "approach"):
                if plan_id:
                    if plan_id not in self.plans:
                        raise ValueError("未知或失效的 plan_id，请重新识别")
                    plan = copy.deepcopy(self.plans[plan_id])
                    if plan["expires_at"] <= time.time():
                        raise ValueError("计划已过期，请重新识别")
                elif not description:
                    raise ValueError("请给出 object 或有效 plan_id")
                if action != "approach" and not self.alignment["confirmed"]:
                    raise ValueError("两指闭合方向尚未核对：先 approach 到上方，核对后用 set_jaw_alignment 保存偏置和 confirmed=true")
            self.cancel.clear()
            self.state = dict(state="RUNNING", run_id=uuid.uuid4().hex, action=action,
                stage="STARTING", message="准备执行；下发命令不等于到位", result=None, started_at=time.time())
            if destination:
                self.state["destination"] = destination
            self.thread = threading.Thread(target=self._execute,
                args=(action, description, backend, plan, destination), daemon=True)
            self.thread.start()
            return self.task_status()

    def _stage(self, name):
        self._not_canceled()
        with self.lock:
            self.state.update(stage=name, message=name)

    def _not_canceled(self):
        if self.cancel.is_set():
            raise RuntimeError("任务已停止，不再发送后续动作")

    def _write(self, group, name, **args):
        with self.issue_lock:
            self._not_canceled()
            return self.primitives.call(group, name, **args)

    def _idle(self):
        state = self.primitives.call("arm", "get_state")
        if state.get("error") or state.get("collision") or state.get("paused") or state.get("fault"):
            raise ValueError("机械臂有报警、碰撞、暂停或通信故障，请先人工处理")
        # RunQueuedCmd describes the queue engine, which may remain on at
        # rest. Use measured motion and joint velocities, like the primitive.
        velocities = finite_vector(state["joint_velocity_deg_s"], 6, "joint velocities")
        if not state.get("enabled") or state["mode"] != 5 or state.get("running") or np.max(np.abs(velocities)) >= .5:
            raise ValueError("机械臂需已使能、退出拖动并静止；本技能不自动使能")
        if state.get("feedback_user") != 0 or state.get("feedback_tool") != 0 or state.get("dashboard_errors"):
            raise ValueError("需有效 User0/Tool0 反馈和 dashboard 状态")
        return state

    def _move(self, pose=None, joints=None, motion="MovL"):
        self._not_canceled()
        if joints is not None:
            run = self._write("arm", "move_joints", joints_deg=joints, velocity_percent=self.config["velocity_percent"])
        else:
            run = self._write("arm", "move_pose", pose_mm_deg=pose, motion=motion, velocity_percent=self.config["velocity_percent"])
        deadline = time.monotonic()+self.config["motion_timeout_s"]
        while run["state"] in ("PENDING", "RUNNING"):
            self._not_canceled()
            if time.monotonic() > deadline:
                raise TimeoutError("运动等待超时")
            run = self.primitives.call("arm", "motion_status", run_id=run["run_id"], wait_s=.2)
            self.cancel.wait(.05)
        if run["state"] != "SUCCEEDED":
            raise RuntimeError("机械臂未确认到位："+str(run))
        self._not_canceled()

    def _gripper(self, action):
        kwargs = dict(speed=self.config["gripper_speed"])
        if action == "grip":
            kwargs["force_percent"] = self.config["force_percent"]
        self._write("gripper", action, **kwargs)
        deadline = time.monotonic()+self.config["gripper_timeout_s"]
        matches = 0
        while time.monotonic() < deadline:
            self._not_canceled()
            s = self.primitives.call("gripper", "get_state")
            if s.get("write_fault") or s.get("error_flags"):
                raise RuntimeError("夹爪反馈故障")
            match = s.get("holding") if action == "grip" else not s.get("moving") and not s.get("holding")
            matches = matches+1 if match else 0
            if matches >= 3:
                return s
            self.cancel.wait(.1)
        raise TimeoutError("夹爪未确认稳定夹持" if action == "grip" else "夹爪未确认停止且释放；不继续下落")

    def _at_ready_pose(self, state):
        pose = finite_vector(state["pose_mm_deg"], 6, "current pose")
        target = self.ready["pose_mm_deg"]
        distance_mm = np.linalg.norm(pose[:3]-target[:3])
        # Match move_pose arrival tolerances and handle equivalent Euler angles.
        actual_rotation = Rotation.from_euler("xyz", pose[3:], degrees=True)
        target_rotation = Rotation.from_euler("xyz", target[3:], degrees=True)
        angle_deg = math.degrees((actual_rotation.inv()*target_rotation).magnitude())
        return distance_mm <= 1.0 and angle_deg <= 1.0

    def _lift_before_ready(self, pose):
        hover_z = self.calibration["z_mm"]+self.config["approach_height_mm"]
        if pose[2] < hover_z-.5:
            if vertical_tilt(pose) > self.config["vertical_tolerance_deg"]:
                raise ValueError("当前处于低位且夹爪不垂直，请先人工抬升后再回准备位")
            raised = [*pose[:2], hover_z, *pose[3:]]
            self.primitives.call("arm", "inverse_kinematics", pose_mm_deg=raised)
            self._move(pose=raised)

    def _home(self):
        self._stage("READY")
        current = self._idle()
        if np.max(np.abs(np.asarray(current["joints_deg"])-self.ready["joints_deg"])) <= .5:
            return
        self._lift_before_ready(current["pose_mm_deg"])
        self._move(joints=self.ready["joints_deg"])

    def _placement_moves(self, destination, hover):
        location = self.place_locations[destination]
        release_z = location["release_z_mm"]
        transit_z = max(hover[2], release_z)
        moves = []
        if transit_z > hover[2]:
            moves.append(("LIFT_TO_TRANSFER_HEIGHT", [*hover[:2], transit_z, *hover[3:]]))
        moves.append(("MOVE_ABOVE_DESTINATION", [*location["xy_mm"], transit_z, *hover[3:]]))
        if release_z < transit_z:
            moves.append(("DESCEND_TO_RELEASE_HEIGHT", [*location["xy_mm"], release_z, *hover[3:]]))
        return moves

    def _require_holding(self):
        state = self.primitives.call("gripper", "get_state")
        if not state.get("holding") or any(state.get(k) for k in ("moving", "error_flags", "write_fault")):
            raise RuntimeError("夹爪未报告稳定夹持，停止后续动作")
        return state

    def _execute(self, action, description, backend, plan, destination):
        motion_started = False
        try:
            if action == "release":
                self._stage("RELEASE")
                result = dict(gripper=self._gripper("open"), arm_moved=False)
            elif action == "home":
                motion_started = True
                self._home()
                result = dict(ready=self.ready, gripper_changed=False)
            elif action == "cycle_from_hover":
                current = self._idle()
                pose = current["pose_mm_deg"]
                grasp_z = self.calibration["z_mm"]
                hover_z = grasp_z+self.config["approach_height_mm"]
                if abs(pose[2]-hover_z) > 2 or vertical_tilt(pose) > self.config["vertical_tolerance_deg"]:
                    raise ValueError("当前必须停在已人工对准的垂直上方待夹位（配置上方 Z ±2 mm）")
                # Validate current XY against the calibrated coverage without
                # re-detecting an object occluded by the gripper itself.
                inv = np.linalg.inv(np.asarray(self.calibration["pixel_to_xy"]))
                pixel = inv @ [*pose[:2], 1.]
                from .geometry import map_pixel
                map_pixel(self.calibration, (pixel[:2]/pixel[2]).tolist())
                grip_state = self.primitives.call("gripper", "get_state")
                if any(grip_state.get(k) for k in ("holding", "moving", "write_fault", "error_flags")):
                    raise ValueError("夹爪必须空闲、无夹持、无故障，才能从待夹位测试")
                hover = pose.copy()
                down = [*pose[:2], grasp_z, *pose[3:]]
                self.primitives.call("arm", "inverse_kinematics", pose_mm_deg=down)
                self.primitives.call("arm", "inverse_kinematics", pose_mm_deg=hover)
                with self.lock:
                    self.state["targets"] = dict(hover_pose_mm_deg=hover, grasp_pose_mm_deg=down)
                motion_started = True
                self._stage("OPEN")
                self._gripper("open")
                self._stage("DESCEND_VERTICAL")
                self._move(pose=down)
                self._stage("GRIP")
                held = self._gripper("grip")
                self._stage("LIFT_VERTICAL")
                self._move(pose=hover)
                held = self.primitives.call("gripper", "get_state")
                if not held.get("holding") or held.get("write_fault") or held.get("error_flags"):
                    raise RuntimeError("抬升后未保持稳定夹持，不继续放回流程")
                self._stage("PLACE_BACK_VERTICAL")
                self._move(pose=down)
                self._stage("RELEASE_AT_ORIGINAL_POSITION")
                released = self._gripper("open")
                self._stage("RETRACT_VERTICAL")
                self._move(pose=hover)
                self._stage("RETURN_READY")
                self._move(joints=self.ready["joints_deg"])
                result = dict(targets=dict(hover_pose_mm_deg=hover, grasp_pose_mm_deg=down),
                    grasp_feedback=held, release_feedback=released, returned_ready=True,
                    object_placed_back=True, meaning="measured holding/release; visually confirm the object returned")
            else:
                current = self._idle()
                # Before any opening or move, reject faults / occupied jaws.
                grip_state = self.primitives.call("gripper", "get_state")
                if grip_state.get("holding") or grip_state.get("moving") or grip_state.get("write_fault") or grip_state.get("error_flags"):
                    raise ValueError("夹爪正在夹持、运动或故障；请先处理，避免抓取前松开已有物体")
                if not self._at_ready_pose(current):
                    self._stage("READY")
                    motion_started = True
                    self._lift_before_ready(current["pose_mm_deg"])
                    self._move(pose=self.ready["pose_mm_deg"], motion="MovJ")
                    if not self._at_ready_pose(self._idle()):
                        raise RuntimeError("反馈显示机械臂尚未到达就绪位")
                    # The arm may have blocked the view used to make the plan.
                    if plan is not None:
                        description, backend = plan["object"], plan["backend"]
                        plan = None
                if plan is None:
                    self._stage("DETECT_AND_PLAN")
                    plan = self.detect_object(description, backend, check_ik=True, internal=True)
                else:
                    self.check_plan(plan)
                placement_moves = []
                if destination:
                    self._stage("CHECK_DESTINATION_IK")
                    placement_moves = self._placement_moves(destination, plan["hover_pose_mm_deg"])
                    for _, pose in placement_moves:
                        self._not_canceled()
                        self.primitives.call("arm", "inverse_kinematics", pose_mm_deg=pose)
                self._not_canceled()
                motion_started = True
                with self.lock:
                    self.state["plan"] = copy.deepcopy(plan)
                if action != "approach":
                    self._stage("OPEN")
                    self._gripper("open")
                self._stage("MOVE_ABOVE")
                self._move(pose=plan["unaligned_hover_pose_mm_deg"], motion="MovJ")
                self._stage("ALIGN_SHORT_AXIS")
                self._move(pose=plan["hover_pose_mm_deg"], motion="MovL")
                if action == "approach":
                    result = dict(plan=plan, at_hover=True, descended=False, gripper_changed=False)
                else:
                    self._stage("DESCEND_VERTICAL")
                    self._move(pose=plan["grasp_pose_mm_deg"], motion="MovL")
                    self._stage("GRIP")
                    self._gripper("grip")
                    self._stage("LIFT_VERTICAL")
                    self._move(pose=plan["hover_pose_mm_deg"], motion="MovL")
                    held = self._require_holding()
                    result = dict(plan=plan, returned_ready=False, at_hover=not destination, gripper=held)
                    if destination:
                        for stage, pose in placement_moves:
                            self._stage(stage)
                            self._move(pose=pose)
                            self._require_holding()
                        self._stage("RELEASE_AT_DESTINATION")
                        result.update(destination=destination, released=True, grasp_feedback=held,
                                      release_pose_mm_deg=placement_moves[-1][1],
                                      gripper=self._gripper("open"))
            with self.lock:
                if self.cancel.is_set():
                    raise RuntimeError("任务已停止")
                self.state.update(state="SUCCEEDED", message="全部步骤已通过反馈确认", result=result, finished_at=time.time())
        except Exception as error:
            stopped = None
            if motion_started and not self.cancel.is_set():
                try:
                    with self.issue_lock:
                        stopped = self.primitives.call("arm", "stop")
                except Exception as stop_error:
                    stopped = dict(error=str(stop_error))
            with self.lock:
                self.state.update(state="CANCELED" if self.cancel.is_set() else "FAILED", message=str(error),
                    stop_result=stopped, finished_at=time.time())
        finally:
            with self.lock:
                ident = self.state["run_id"]
                (self.artifacts/(ident+"-run.json")).write_text(json.dumps(self.state, ensure_ascii=False, indent=2)+"\n")

    def stop(self):
        self.cancel.set()
        with self.issue_lock:
            result = self.primitives.call("arm", "stop")
        with self.lock:
            if self.state["state"] == "RUNNING":
                self.state.update(state="CANCELED", message="已请求停止；不自动回位或松爪")
        return dict(arm=result, gripper="No verified gripper stop command; no open/reverse sent", task=self.task_status())

    def close(self):
        if self.active():
            self.stop()
            self.thread.join(timeout=3)
