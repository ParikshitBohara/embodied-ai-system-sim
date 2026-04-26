# -*- coding: utf-8 -*-

from pathlib import Path
from typing import Dict, Tuple

import cv2
import numpy as np
import pybullet as p


def get_top_down_camera_config() -> Dict[str, object]:
    """
    Create a top-down camera configuration for the simulation scene.

    The camera is used to capture RGB, depth, and segmentation data.
    """

    width = 640
    height = 480

    camera_target = [0.55, 0.0, 0.62]
    camera_distance = 0.9
    yaw = 90
    pitch = -88
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

    near_val = 0.01
    far_val = 2.0
    fov = 60
    aspect = width / height

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
        "near_val": near_val,
        "far_val": far_val,
        "fov": fov,
        "camera_target": camera_target,
        "camera_distance": camera_distance,
        "yaw": yaw,
        "pitch": pitch,
        "roll": roll,
    }


def capture_rgb_image(
    width: int,
    height: int,
    view_matrix,
    projection_matrix,
) -> np.ndarray:
    """
    Capture only the RGB image from the PyBullet camera.

    This function is kept for compatibility with other files that may only need RGB.
    """

    frame = capture_camera_frame(
        width=width,
        height=height,
        view_matrix=view_matrix,
        projection_matrix=projection_matrix,
    )

    return frame["rgb"]


def capture_camera_frame(
    width: int,
    height: int,
    view_matrix,
    projection_matrix,
    near_val: float = 0.01,
    far_val: float = 2.0,
) -> Dict[str, np.ndarray]:
    """
    Capture RGB, depth, depth in meters, and segmentation images from PyBullet.
    """

    _, _, rgba_pixels, depth_pixels, segmentation_pixels = p.getCameraImage(
        width=width,
        height=height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL,
    )

    rgba_array = np.array(rgba_pixels, dtype=np.uint8).reshape(height, width, 4)
    depth_array = np.array(depth_pixels, dtype=np.float32).reshape(height, width)
    segmentation_array = np.array(segmentation_pixels, dtype=np.int32).reshape(height, width)

    depth_meters = depth_buffer_to_meters(
        depth_buffer=depth_array,
        near_val=near_val,
        far_val=far_val,
    )

    return {
        "rgb": rgba_array[:, :, :3],
        "depth": depth_array,
        "depth_meters": depth_meters,
        "segmentation": segmentation_array,
    }


def depth_buffer_to_meters(
    depth_buffer: np.ndarray,
    near_val: float,
    far_val: float,
) -> np.ndarray:
    """
    Convert PyBullet depth buffer values into real depth values in meters.

    PyBullet depth values are normalized between 0 and 1.
    """

    return far_val * near_val / (
        far_val - (far_val - near_val) * depth_buffer
    )


def pixel_to_world(
    pixel_x: int,
    pixel_y: int,
    depth_value: float,
    width: int,
    height: int,
    view_matrix,
    projection_matrix,
) -> Tuple[float, float, float]:
    """
    Convert a 2D image pixel and its depth buffer value into a 3D world coordinate.
    """

    if pixel_x < 0 or pixel_x >= width:
        raise ValueError(f"pixel_x is outside the image width: {pixel_x}")

    if pixel_y < 0 or pixel_y >= height:
        raise ValueError(f"pixel_y is outside the image height: {pixel_y}")

    if depth_value < 0.0 or depth_value > 1.0:
        raise ValueError(f"depth_value must be between 0 and 1: {depth_value}")

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


def save_camera_frame(
    frame: Dict[str, np.ndarray],
    output_folder: str = "outputs",
) -> None:
    """
    Save RGB, depth, and segmentation images to an output folder.

    This is useful for debugging, testing, and report screenshots.
    """

    required_keys = ["rgb", "depth"]
    for key in required_keys:
        if key not in frame:
            raise KeyError(f"Missing required frame data: {key}")

    output_path = Path(output_folder)
    output_path.mkdir(parents=True, exist_ok=True)

    rgb_image = frame["rgb"]

    # Prefer real depth in meters if it exists.
    depth_image = frame.get("depth_meters", frame["depth"])

    # Save RGB image.
    # OpenCV uses BGR format, so RGB must be converted to BGR before saving.
    rgb_bgr = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
    cv2.imwrite(str(output_path / "rgb.png"), rgb_bgr)

    # Save depth image as a visible grayscale image.
    depth_normalized = cv2.normalize(
        depth_image,
        None,
        alpha=0,
        beta=255,
        norm_type=cv2.NORM_MINMAX,
    )
    depth_uint8 = depth_normalized.astype(np.uint8)
    cv2.imwrite(str(output_path / "depth.png"), depth_uint8)

    # Save segmentation image if available.
    if "segmentation" in frame:
        segmentation_image = frame["segmentation"]

        segmentation_normalized = cv2.normalize(
            segmentation_image,
            None,
            alpha=0,
            beta=255,
            norm_type=cv2.NORM_MINMAX,
        )
        segmentation_uint8 = segmentation_normalized.astype(np.uint8)
        cv2.imwrite(str(output_path / "segmentation.png"), segmentation_uint8)

    print(f"Camera frame saved to: {output_path.resolve()}")


def capture_and_save_camera_frame(
    camera_config: Dict[str, object],
    output_folder: str = "outputs",
) -> Dict[str, np.ndarray]:
    """
    Capture a full camera frame and save it to files.
    """

    frame = capture_camera_frame(
        width=int(camera_config["width"]),
        height=int(camera_config["height"]),
        view_matrix=camera_config["view_matrix"],
        projection_matrix=camera_config["projection_matrix"],
        near_val=float(camera_config["near_val"]),
        far_val=float(camera_config["far_val"]),
    )

    save_camera_frame(frame, output_folder)

    return frame
