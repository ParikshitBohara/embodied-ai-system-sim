# -*- coding: utf-8 -*-

from typing import Dict

import numpy as np
import pybullet as p


def get_top_down_camera_config() -> Dict[str, object]:

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


def capture_camera_frame(
    width: int,
    height: int,
    view_matrix,
    projection_matrix,
) -> Dict[str, np.ndarray]:

    _, _, rgba_pixels, depth_pixels, _ = p.getCameraImage(
        width=width,
        height=height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL,
    )

    rgba_array = np.array(rgba_pixels, dtype=np.uint8).reshape(height, width, 4)
    depth_array = np.array(depth_pixels, dtype=np.float32).reshape(height, width)

    return {
        "rgb": rgba_array[:, :, :3],
        "depth": depth_array,
    }


def pixel_to_world(
    pixel_x: int,
    pixel_y: int,
    depth_value: float,
    width: int,
    height: int,
    view_matrix,
    projection_matrix,
) -> tuple[float, float, float]:

    normalized_x = (2.0 * pixel_x / width) - 1.0
    normalized_y = 1.0 - (2.0 * pixel_y / height)
    normalized_z = (2.0 * depth_value) - 1.0

    projection = np.array(projection_matrix, dtype=np.float64).reshape(4, 4, order="F")
    view = np.array(view_matrix, dtype=np.float64).reshape(4, 4, order="F")
    inverse_view_projection = np.linalg.inv(projection @ view)

    clip_coordinates = np.array(
        [normalized_x, normalized_y, normalized_z, 1.0],
        dtype=np.float64,
    )
    world_coordinates = inverse_view_projection @ clip_coordinates
    world_coordinates /= world_coordinates[3]

    return tuple(float(value) for value in world_coordinates[:3])
