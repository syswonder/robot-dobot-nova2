"""Calibrated planar geometry. No hardware or model calls."""
from __future__ import annotations

import math

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


def finite_vector(value, size, name):
    array = np.asarray(value, dtype=float)
    if array.shape != (size,) or not np.isfinite(array).all():
        raise ValueError(f"{name} must contain {size} finite numbers")
    return array


def vertical_tilt(pose):
    axis = Rotation.from_euler("xyz", finite_vector(pose, 6, "pose")[3:], degrees=True).apply([0, 0, 1])
    return float(np.degrees(np.arctan2(np.linalg.norm(axis[:2]), -axis[2])))


def map_pixel(calibration, point, inside=True):
    p = finite_vector(point, 2, "pixel")
    if inside and cv2.pointPolygonTest(np.asarray(calibration["valid_pixel_polygon"], np.float32), tuple(p), False) < 0:
        raise ValueError("目标超出标定覆盖区域；请将物体移入标定区域后重新识别")
    H = np.asarray(calibration["pixel_to_xy"], float)
    if H.shape != (3, 3) or not np.isfinite(H).all():
        raise ValueError("无效标定矩阵")
    mapped = H @ [*p, 1.]
    if not np.isfinite(mapped).all() or abs(mapped[2]) < 1e-9:
        raise ValueError("像素映射不稳定")
    return mapped[:2] / mapped[2]


def ordered_rectangle(points, width, height):
    points = np.asarray(points, dtype=float)
    if points.shape != (4, 2) or not np.isfinite(points).all():
        raise ValueError("检测结果必须包含顶面矩形的四个角点")
    if (points < 0).any() or (points[:, 0] >= width).any() or (points[:, 1] >= height).any():
        raise ValueError("检测角点超出原图范围")
    hull = cv2.convexHull(points.astype(np.float32)).reshape(-1, 2)
    if len(hull) != 4 or cv2.contourArea(hull) < 16:
        raise ValueError("顶面四角退化或不构成凸四边形")
    return hull.astype(float)


def center_of_face(points):
    # Intersection of diagonals: unlike bounding-box centre, correct for a
    # projectively imaged planar rectangle.
    a, b, c, d = np.asarray(points, float)
    t, _ = np.linalg.solve(np.column_stack((c-a, b-d)), b-a)
    return a+t*(c-a)


def pixel_pair(points, width, height):
    points = np.asarray(points, dtype=float)
    if points.shape != (2, 2) or not np.isfinite(points).all():
        raise ValueError("侧夹需要同一截面上的两个有效像素点")
    if (points < 0).any() or (points[:, 0] >= width).any() or (points[:, 1] >= height).any():
        raise ValueError("侧夹点超出原图范围")
    if np.linalg.norm(points[1]-points[0]) < 2:
        raise ValueError("两个侧夹点过近")
    return points


def equivalent_yaw(axis, offset_deg, reference_deg):
    yaw = math.degrees(math.atan2(axis[1], axis[0])) - offset_deg
    # Two-finger closure is symmetric under 180 degrees.
    distance = (yaw-reference_deg+90) % 180-90
    return abs(distance), (reference_deg+distance+180) % 360-180


def side_grasp_geometry(calibration, points, yaw_offset_deg, reference_yaw_deg):
    # Map both sides first so that perspective does not shift the midpoint.
    world = np.asarray([map_pixel(calibration, point) for point in points])
    axis = world[1]-world[0]
    width_mm = float(np.linalg.norm(axis))
    if not 2 <= width_mm <= 200:
        raise ValueError(f"侧夹估计宽度异常（{width_mm:.1f} mm）")
    xy = np.mean(world, axis=0)
    h_inv = np.linalg.inv(np.asarray(calibration["pixel_to_xy"], float))
    pixel = h_inv @ [*xy, 1.]
    if not np.isfinite(pixel).all() or abs(pixel[2]) < 1e-9:
        raise ValueError("侧夹中心投影不稳定")
    center = pixel[:2]/pixel[2]
    map_pixel(calibration, center)
    _, yaw = equivalent_yaw(axis, yaw_offset_deg, reference_yaw_deg)
    return dict(center_pixel=center.tolist(), xy_mm=xy.tolist(),
                closing_axis_pixels=points.tolist(), closing_axis_xy=(axis/width_mm).tolist(),
                closing_width_mm=width_mm, yaw_deg=yaw, jaw_axis_offset_deg=yaw_offset_deg)


def target_geometry(calibration, corners, width, height, yaw_offset_deg=0., reference_yaw_deg=0., square_ratio=1.12):
    corners = ordered_rectangle(corners, width, height)
    center = center_of_face(corners)
    xy = map_pixel(calibration, center)
    # Only the commanded centre must be inside the calibrated polygon. A
    # face can straddle its boundary; edge points are used locally for yaw,
    # never as commanded robot positions.
    world = np.asarray([map_pixel(calibration, p, inside=False) for p in corners])
    H = np.asarray(calibration["pixel_to_xy"], float)
    denominators = np.column_stack((corners, np.ones(4))) @ H[2]
    if np.min(denominators)*np.max(denominators) <= 0:
        raise ValueError("物体顶面跨越映射无穷线")
    # Opposite-edge midpoints define the two closing axes in ROBOT space.
    axes = [(world[1]+world[2]-world[0]-world[3])/2,
            (world[2]+world[3]-world[0]-world[1])/2]
    lengths = [float(np.linalg.norm(v)) for v in axes]
    if min(lengths) < 2 or max(lengths) > 200:
        raise ValueError(f"物体估计尺寸异常（{lengths} mm），检查顶面框和标定")
    square = max(lengths)/min(lengths) <= square_ratio
    indices = [0, 1] if square else [int(np.argmin(lengths))]
    _, yaw, axis = min((*equivalent_yaw(axes[i], yaw_offset_deg, reference_yaw_deg), i)
                       for i in indices)
    return dict(center_pixel=center.tolist(), xy_mm=xy.tolist(), corners_pixel=corners.tolist(),
                corners_xy_mm=world.tolist(), dimensions_mm=lengths, square_ambiguous=square,
                closing_axis_xy=(axes[axis]/np.linalg.norm(axes[axis])).tolist(),
                closing_width_mm=lengths[axis], yaw_deg=yaw, jaw_axis_offset_deg=yaw_offset_deg)
