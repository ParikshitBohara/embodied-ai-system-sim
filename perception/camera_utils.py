# -*- coding: utf-8 -*-

from typing import Dict, Tuple

import numpy as np
import pybullet as p


def get_top_down_camera_config() -> Dict[str, object]:
    """
    Return a simple fixed camera configuration for observing the table scene.
    """
    width = 640
    height = 480

    camera_target = [0.5, 0.0, 0.62]
    camera_distance = 0.55
    yaw = 90
    pitch = -70
    roll = 0
    up_axis_index = 2

    view_matrix = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=camera_target,
        distance=camera_distance,
        yaw=yaw,
        pitch=pitch,
        roll=roll,
        upAxisIndex=up_axis_index,
    )

    aspect = width / height
    near_val = 0.01
    far_val = 2.0
    fov = 60

    projection_matrix = p.computeProjectionMatrixFOV(
        fov=fov,
        aspect=aspect,
        nearVal=near_val,
        farVal=far_val,
    )

    return {
        "width": width,
        "height": height,
        "view_matrix": view_matrix,
        "projection_matrix": projection_matrix,
    }


def capture_rgb_image(
    width: int,
    height: int,
    view_matrix,
    projection_matrix,
) -> np.ndarray:
    """
    Capture an RGB image from the simulator camera.

    Returns:
        RGB image as a NumPy array of shape (height, width, 3).
    """
    _, _, rgba_pixels, _, _ = p.getCameraImage(
        width=width,
        height=height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL,
    )

    rgba_array = np.array(rgba_pixels, dtype=np.uint8).reshape(height, width, 4)
    rgb_image = rgba_array[:, :, :3]

    return rgb_image
