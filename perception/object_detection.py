from typing import Dict, List

import cv2
import numpy as np

from perception.camera_utils import pixel_to_world

COLOR_RANGES = {
    "red": [
        (np.array([0, 120, 70]), np.array([10, 255, 255])),
        (np.array([170, 120, 70]), np.array([180, 255, 255])),
    ],
    "blue": [
        (np.array([95, 80, 60]), np.array([130, 255, 255])),
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


def _mask_for_color(hsv_image: np.ndarray, color_name: str) -> np.ndarray:
    mask = np.zeros(hsv_image.shape[:2], dtype=np.uint8)
    for lower, upper in COLOR_RANGES[color_name]:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv_image, lower, upper))

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
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


def detect_sortable_objects(rgb_image: np.ndarray) -> List[Dict[str, object]]:
    hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
    results = []

    for color_name, class_name in CLASS_BY_COLOR.items():
        mask = _mask_for_color(hsv, color_name)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 120:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if area > 6000:
                continue

            center_x = x + w // 2
            center_y = y + h // 2
            results.append(
                {
                    "class_name": class_name,
                    "color": color_name,
                    "shape": SHAPE_BY_CLASS[class_name],
                    "observed_shape": _shape_name(contour, (x, y, w, h)),
                    "bbox": (x, y, x + w, y + h),
                    "center": (center_x, center_y),
                    "confidence": 1.0,
                    "area": float(area),
                }
            )

    results.sort(key=lambda x: x["area"], reverse=True)
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
    x1, y1, x2, y2 = detection["bbox"]
    depth_region = depth_image[y1:y2, x1:x2]
    valid_depth_values = depth_region[(depth_region > 0.0) & (depth_region < 1.0)]
    if valid_depth_values.size == 0:
        depth_value = float(depth_image[center_y, center_x])
    else:
        depth_value = float(np.median(valid_depth_values))

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
        x1, y1, x2, y2 = detection["bbox"]
        depth_region = depth_image[y1:y2, x1:x2]
        valid_depth_values = depth_region[(depth_region > 0.0) & (depth_region < 1.0)]
        if valid_depth_values.size == 0:
            depth_value = float(depth_image[center_y, center_x])
        else:
            depth_value = float(np.median(valid_depth_values))

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
