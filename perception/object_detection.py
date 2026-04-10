from typing import Dict, List

import cv2
import numpy as np


def detect_red_cube(rgb_image: np.ndarray) -> List[Dict[str, object]]:
    """Detect red cube in an RGB image."""
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
