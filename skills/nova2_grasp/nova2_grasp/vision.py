"""Exact calibrated camera frames and interchangeable VLM / YOLO-OBB detectors."""
from __future__ import annotations

import base64
import json
import os
from pathlib import Path
import time

import cv2
import numpy as np
import requests

from .geometry import ordered_rectangle, pixel_pair


class Camera:
    def __init__(self, config, primitives):
        self.primitives = primitives
        self.max_age = config["camera_max_frame_age_s"]

    def capture(self, calibration):
        started = time.monotonic()
        data = self.primitives.call("camera", "snapshot")
        # Validate the actual frame, not metadata from a separate state query.
        age = data.get("age_s")
        if isinstance(age,bool) or not isinstance(age,(int,float)) or not np.isfinite(age) or age < 0 or age+time.monotonic()-started > self.max_age:
            raise ValueError("相机画面已过期或时间信息无效")
        if data.get("camera_key") != calibration["camera_key"]:
            raise ValueError("相机模式与标定不一致")
        if data.get("mime_type") != "image/jpeg":
            raise ValueError("相机原语必须返回 JPEG 图像")
        encoded = data.get("jpeg_base64", "")
        if not isinstance(encoded,str) or len(encoded)>14000000:
            raise ValueError("相机图像数据长度无效")
        try:
            jpeg = base64.b64decode(encoded,validate=True)
        except (ValueError,TypeError) as error:
            raise ValueError("相机 JPEG 编码无效") from error
        image = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR) if jpeg else None
        if image is None or [image.shape[1], image.shape[0]] != calibration["image_size"]:
            raise ValueError("相机图像分辨率与标定不一致")
        return dict(image=image,jpeg=jpeg,frame_id=data.get("frame_id"),
                    camera_key=data["camera_key"],captured_at=time.time())


def json_reply(text):
    text = text.strip()
    if text.startswith("```"):
        text = text.split("\n", 1)[1].rsplit("```", 1)[0].strip()
    result = json.loads(text)
    if not isinstance(result, dict) or not isinstance(result.get("found"), bool):
        raise ValueError("模型必须返回包含 found 布尔值的 JSON 对象")
    return result


class VlmDetector:
    def __init__(self, config):
        self.config = config

    def detect(self, frame, description):
        c = self.config
        url = c.get("vlm_base_url") or os.environ.get("VLM_BASE_URL", "")
        model = c.get("vlm_model") or os.environ.get("VLM_MODEL", "")
        key = c.get("vlm_api_key") or os.environ.get("VLM_API_KEY", "")
        if not url or not model:
            raise ValueError("VLM 未配置：设置 VLM_BASE_URL / VLM_MODEL / VLM_API_KEY")
        endpoint = url.rstrip("/")
        if not endpoint.endswith("/chat/completions"):
            endpoint += "/chat/completions"
        height, width = frame["image"].shape[:2]
        prompt = (
            f"Locate exactly one object matching this description: {description!r}. "
            f"The supplied RGB image is {width} by {height} pixels. "
            "Return JSON only, with found, label, confidence, bbox_xyxy, and grasp_geometry. "
            "Use absolute pixels in the original image, not normalized coordinates. "
            "For a block with a visible flat rectangular top, use grasp_geometry=\"flat_top\" "
            "and top_corners_px=[[x1,y1],[x2,y2],[x3,y3],[x4,y4]] in cyclic order on that face. "
            "For a slender rounded or tapered body (e.g. a carrot), use grasp_geometry=\"side_pair\" "
            "and side_points_px=[[x1,y1],[x2,y2]]. Pick two visible opposite body edges at the same "
            "cross-section, with the segment approximately perpendicular to the local long axis. "
            "Use a substantial section near the middle where both jaws have clearance. Avoid "
            "the narrow tip, stem, leaves, shadows and nearby objects. The midpoint is the "
            "gripper center and the line joining the points is the two-jaw closing direction. "
            "Do not invent flat corners for curved objects. Do not estimate height or depth. "
            "If absent, ambiguous, occluded at the grasp section, or reliable points cannot be determined, "
            "return {\"found\":false,\"reason\":\"...\"}."
        )
        payload = dict(model=model, temperature=0, messages=[dict(role="user", content=[
            dict(type="text", text=prompt), dict(type="image_url", image_url=dict(
                url="data:image/jpeg;base64,"+base64.b64encode(frame["jpeg"]).decode()))])])
        # No automatic replay of a failed model request; never log credentials.
        response = requests.post(endpoint, headers={"Authorization": "Bearer "+key},
                                 json=payload, timeout=c["vlm_timeout_s"])
        if response.status_code != 200:
            raise ValueError(f"VLM 请求失败 HTTP {response.status_code}（未输出凭据或响应正文）")
        body = response.json()
        content = body["choices"][0]["message"]["content"]
        if isinstance(content, list):
            content = "".join(v.get("text", "") for v in content if isinstance(v, dict))
        result = json_reply(content)
        result["backend"] = "vlm"
        result["model"] = model
        return result


class YoloDetector:
    def __init__(self, config):
        self.config, self.model = config, None

    def detect(self, frame, description):
        weights = Path(self.config["yolo_model_path"]).expanduser()
        if not self.config["yolo_model_path"] or not weights.is_file():
            raise ValueError("YOLO 后端需要本地 YOLO-OBB 权重 yolo_model_path；不自动下载或退回其他后端")
        if self.model is None:
            try:
                from ultralytics import YOLO
            except ImportError as error:
                raise ValueError("本技能环境未安装 ultralytics，请按 README 安装可选 YOLO 依赖") from error
            self.model = YOLO(str(weights))
        aliases = json.loads(self.config.get("yolo_class_aliases_json", "{}"))
        wanted = aliases.get(description, description)
        wanted = [wanted] if isinstance(wanted, str) else wanted
        result = self.model.predict(frame["image"], conf=self.config["min_confidence"], verbose=False)[0]
        if result.obb is None:
            raise ValueError("YOLO 权重必须输出 OBB；普通水平框不能确定物体短轴")
        candidates = []
        for corners, score, cls in zip(result.obb.xyxyxyxy.cpu().numpy(), result.obb.conf.cpu().numpy(), result.obb.cls.cpu().numpy()):
            label = str(result.names[int(cls)])
            if label in wanted:
                candidates.append(dict(found=True, label=label, confidence=float(score),
                    top_corners_px=corners.tolist(), backend="yolo", model=str(weights)))
        if len(candidates) != 1:
            return dict(found=False, backend="yolo", reason=f"匹配 {len(candidates)} 个目标，请使用明确的类别名称/别名并确保仅有一个目标")
        return candidates[0]


def validate_detection(detection, frame, min_confidence):
    if not detection.get("found"):
        raise ValueError("未找到唯一且可定位的目标："+str(detection.get("reason", "not found")))
    confidence = detection.get("confidence")
    if isinstance(confidence, bool) or not isinstance(confidence, (int, float)) or not np.isfinite(confidence) or not min_confidence <= confidence <= 1:
        raise ValueError("目标置信度不足或格式无效")
    height, width = frame["image"].shape[:2]
    geometry = detection.get("grasp_geometry", "flat_top")
    if geometry == "side_pair":
        points = pixel_pair(detection.get("side_points_px"), width, height)
        return {**detection, "grasp_geometry": geometry, "side_points_px": points.tolist()}
    if geometry != "flat_top":
        raise ValueError("不支持的识别抓取几何类型")
    corners = ordered_rectangle(detection.get("top_corners_px"), width, height)
    return {**detection, "grasp_geometry": geometry, "top_corners_px": corners.tolist()}


def save_overlay(frame, detection, geometry, path):
    image = frame["image"].copy()
    if "corners_pixel" in geometry:
        corners = np.rint(geometry["corners_pixel"]).astype(np.int32)
        cv2.polylines(image, [corners], True, (40, 210, 60), 2)
    else:
        for point in np.rint(geometry["closing_axis_pixels"]).astype(int):
            cv2.circle(image, tuple(point), 5, (40, 210, 60), -1)
    center = np.rint(geometry["center_pixel"]).astype(int)
    cv2.drawMarker(image, tuple(center), (20, 160, 255), cv2.MARKER_CROSS, 22, 2)
    # Project the selected ROBOT closing axis back through the homography in
    # controller.py, then draw exact pixel endpoints supplied in geometry.
    if "closing_axis_pixels" in geometry:
        a, b = np.rint(geometry["closing_axis_pixels"]).astype(int)
        cv2.arrowedLine(image, tuple(a), tuple(b), (0, 220, 255), 3, tipLength=.2)
    label = detection.get('label', 'object')
    label = label if isinstance(label, str) and label.isascii() else 'target'
    text = f"{detection['backend']} | {label} | yaw {geometry['yaw_deg']:.1f} deg"
    cv2.putText(image, text, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 255), 2)
    if not cv2.imwrite(str(path), image):
        raise OSError("不能保存识别叠加图")
