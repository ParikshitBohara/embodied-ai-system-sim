from typing import Dict, List

import cv2
import numpy as np

from perception.camera_utils import pixel_to_world


def detect_red_cube(rgb_image: np.ndarray) -> List[Dict[str, object]]:

    results = []

    hsv = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)

    red_lower_1 = np.array([0, 120, 70])
    red_upper_1 = np.array([10, 255, 255])

    red_lower_2 = np.array([170, 120, 70])
    red_upper_2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(hsv, red_lower_1, red_upper_1)
    mask2 = cv2.inRange(hsv, red_lower_2, red_upper_2)
    mask = mask1 + mask2

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)

        if area < 150:
            continue

        x, y, w, h = cv2.boundingRect(cnt)
        center_x = x + w // 2
        center_y = y + h // 2

        obj = {
            "class_name": "red_cube",
            "bbox": (x, y, x + w, y + h),
            "center": (center_x, center_y),
            "confidence": 1.0,
            "area": float(area)
        }

        results.append(obj)

    results.sort(key=lambda x: x["area"], reverse=True)
    return results


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
