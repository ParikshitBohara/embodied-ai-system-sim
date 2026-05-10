from typing import Dict, List

import cv2
import numpy as np

from perception.camera_utils import pixel_to_world

MIN_CONTOUR_AREA = 150.0
MAX_CONTOUR_AREA = 6500.0
MIN_SOLIDITY = 0.65
MIN_EXTENT = 0.35

COLOR_RANGES = {
    "red": [
        (np.array([0, 95, 50]), np.array([12, 255, 255])),
        (np.array([168, 95, 50]), np.array([180, 255, 255])),
    ],
    "blue": [
        (np.array([90, 60, 45]), np.array([135, 255, 255])),
    ],
    "yellow": [
        (np.array([18, 80, 80]), np.array([40, 255, 255])),
    ],
}
CLASS_BY_COLOR = {
    "red": "red_cube",
    "blue": "blue_sphere",
    "yellow": "yellow_rectangle",
}
SHAPE_BY_CLASS = {
    "red_cube": "square",
    "blue_sphere": "sphere",
    "yellow_rectangle": "rectangle",
}
EXPECTED_AREA_BY_CLASS = {
    "red_cube": (300.0, 1800.0),
    "blue_sphere": (150.0, 1200.0),
    "yellow_rectangle": (400.0, 2400.0),
}


def _mask_for_color(hsv_image: np.ndarray, color_name: str) -> np.ndarray:
    mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
    for lower, upper in COLOR_RANGES[color_name]:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv_image, lower, upper))

    open_kernel = np.ones((3, 3), np.uint8)
    close_kernel = np.ones((5, 5), np.uint8)
    mask = cv2.medianBlur(mask, 5)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel, iterations=2)
    return mask


def _shape_name(contour: np.ndarray, bbox: tuple[int, int, int, int]) -> str:
    area = cv2.contourArea(contour)
    _, _, width, height = bbox
    aspect_ratio = width / height if height else 0.0
    perimeter = cv2.arcLength(contour, True)
    circularity = 4.0 * np.pi * area / (perimeter * perimeter) if perimeter else 0.0

    if circularity > 0.72:
        return "sphere"
    if aspect_ratio > 1.25 or aspect_ratio < 0.8:
        return "rectangle"
    return "square"


def _safe_rgb_image(rgb_image: np.ndarray) -> np.ndarray:
    if rgb_image.ndim != 3 or rgb_image.shape[2] < 3:
        raise ValueError("rgb_image must have shape (height, width, 3)")

    rgb = rgb_image[:, :, :3]
    if rgb.dtype != np.uint8:
        rgb = np.clip(rgb, 0, 255).astype(np.uint8)

    return np.ascontiguousarray(rgb)


def _area_score(area: float, class_name: str) -> float:
    min_area, max_area = EXPECTED_AREA_BY_CLASS[class_name]
    if area < min_area:
        return max(0.0, area / min_area)
    if area > max_area:
        return max(0.0, max_area / area)
    return 1.0


def _contour_confidence(
    area: float,
    solidity: float,
    extent: float,
    class_name: str,
) -> float:
    area_component = _area_score(area, class_name)
    solidity_component = max(0.0, min(1.0, solidity))
    extent_component = max(0.0, min(1.0, extent))
    return round(
        0.5 * area_component
        + 0.3 * solidity_component
        + 0.2 * extent_component,
        3,
    )


def _detection_from_contour(
    contour: np.ndarray,
    color_name: str,
    class_name: str,
) -> Dict[str, object] | None:
    area = float(cv2.contourArea(contour))
    if area < MIN_CONTOUR_AREA or area > MAX_CONTOUR_AREA:
        return None

    x, y, w, h = cv2.boundingRect(contour)
    if w == 0 or h == 0:
        return None

    bbox_area = float(w * h)
    hull = cv2.convexHull(contour)
    hull_area = float(cv2.contourArea(hull))
    solidity = area / hull_area if hull_area else 0.0
    extent = area / bbox_area if bbox_area else 0.0
    if solidity < MIN_SOLIDITY or extent < MIN_EXTENT:
        return None

    moments = cv2.moments(contour)
    if moments["m00"]:
        center_x = int(moments["m10"] / moments["m00"])
        center_y = int(moments["m01"] / moments["m00"])
    else:
        center_x = x + w // 2
        center_y = y + h // 2

    return {
        "class_name": class_name,
        "color": color_name,
        "shape": SHAPE_BY_CLASS[class_name],
        "observed_shape": _shape_name(contour, (x, y, w, h)),
        "bbox": (x, y, x + w, y + h),
        "center": (center_x, center_y),
        "confidence": _contour_confidence(area, solidity, extent, class_name),
        "area": area,
        "solidity": round(solidity, 3),
        "extent": round(extent, 3),
    }


def _depth_value_for_detection(
    depth_image: np.ndarray,
    detection: Dict[str, object],
) -> float:
    center_x, center_y = detection["center"]
    x1, y1, x2, y2 = detection["bbox"]
    depth_region = depth_image[y1:y2, x1:x2]
    valid_depth_values = depth_region[
        np.isfinite(depth_region)
        & (depth_region > 0.0)
        & (depth_region < 1.0)
    ]
    if valid_depth_values.size == 0:
        return float(depth_image[center_y, center_x])

    return float(np.median(valid_depth_values))


def detect_sortable_objects(rgb_image: np.ndarray) -> List[Dict[str, object]]:
    rgb = _safe_rgb_image(rgb_image)
    blurred_rgb = cv2.GaussianBlur(rgb, (5, 5), 0)
    hsv = cv2.cvtColor(blurred_rgb, cv2.COLOR_RGB2HSV)
    results = []

    for color_name, class_name in CLASS_BY_COLOR.items():
        mask = _mask_for_color(hsv, color_name)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        class_results = []

        for contour in contours:
            detection = _detection_from_contour(contour, color_name, class_name)
            if detection is not None:
                class_results.append(detection)

        class_results.sort(
            key=lambda detection: (detection["confidence"], detection["area"]),
            reverse=True,
        )
        results.extend(class_results[:1])

    results.sort(key=lambda x: (x["confidence"], x["area"]), reverse=True)
    return results


def detect_red_cube(rgb_image: np.ndarray) -> List[Dict[str, object]]:
    return [
        detection
        for detection in detect_sortable_objects(rgb_image)
        if detection["class_name"] == "red_cube"
    ]


def estimate_red_cube_world_position(
    rgb_image: np.ndarray,
    depth_image: np.ndarray,
    camera_config: Dict[str, object],
    cube_center_z: float,
) -> Dict[str, object] | None:

    detections = detect_red_cube(rgb_image)
    if not detections:
        return None

    detection = detections[0]
    center_x, center_y = detection["center"]
    depth_value = _depth_value_for_detection(depth_image, detection)

    world_x, world_y, _ = pixel_to_world(
        pixel_x=center_x,
        pixel_y=center_y,
        depth_value=depth_value,
        width=int(camera_config["width"]),
        height=int(camera_config["height"]),
        view_matrix=camera_config["view_matrix"],
        projection_matrix=camera_config["projection_matrix"],
    )

    return {
        **detection,
        "world_position": (world_x, world_y, cube_center_z),
        "depth_value": depth_value,
    }


def estimate_sortable_object_world_positions(
    rgb_image: np.ndarray,
    depth_image: np.ndarray,
    camera_config: Dict[str, object],
    object_center_z: float,
) -> List[Dict[str, object]]:
    detections = detect_sortable_objects(rgb_image)
    estimates = []

    for detection in detections:
        center_x, center_y = detection["center"]
        depth_value = _depth_value_for_detection(depth_image, detection)

        world_x, world_y, _ = pixel_to_world(
            pixel_x=center_x,
            pixel_y=center_y,
            depth_value=depth_value,
            width=int(camera_config["width"]),
            height=int(camera_config["height"]),
            view_matrix=camera_config["view_matrix"],
            projection_matrix=camera_config["projection_matrix"],
        )
        estimates.append(
            {
                **detection,
                "world_position": (world_x, world_y, object_center_z),
                "depth_value": depth_value,
            }
        )

    return estimates
