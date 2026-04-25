import pybullet as p

from environment.scene import TABLE_SURFACE_Z
from perception.camera_utils import capture_camera_frame, get_top_down_camera_config
from perception.object_detection import (
    estimate_red_cube_world_position,
    estimate_sortable_object_world_positions,
)
from robot.control import (
    move_end_effector_to_position,
    move_to_home,
    step_simulation,
)
from robot.gripper import SimulatedGripper


APPROACH_HEIGHT = 0.25
GRASP_HEIGHT_OFFSET = 0.12
MOVE_STEPS = 180
CUBE_HALF_EXTENT = 0.025
SORT_ORDER = ("red_cube", "blue_sphere", "yellow_rectangle")


def _body_position(body_id: int) -> tuple[float, float, float]:
    position, _ = p.getBasePositionAndOrientation(body_id)
    return position


def perceive_cube_position() -> tuple[float, float, float] | None:
    camera_config = get_top_down_camera_config()
    camera_frame = capture_camera_frame(
        width=int(camera_config["width"]),
        height=int(camera_config["height"]),
        view_matrix=camera_config["view_matrix"],
        projection_matrix=camera_config["projection_matrix"],
    )
    detection = estimate_red_cube_world_position(
        rgb_image=camera_frame["rgb"],
        depth_image=camera_frame["depth"],
        camera_config=camera_config,
        cube_center_z=TABLE_SURFACE_Z + CUBE_HALF_EXTENT,
    )

    if detection is None:
        print("[WARN] OpenCV detector did not find the red cube.")
        return None

    print(
        "[INFO] OpenCV detected red_cube at "
        f"pixel={detection['center']} world={detection['world_position']}"
    )
    return detection["world_position"]


def perceive_sortable_objects() -> list[dict[str, object]]:
    camera_config = get_top_down_camera_config()
    camera_frame = capture_camera_frame(
        width=int(camera_config["width"]),
        height=int(camera_config["height"]),
        view_matrix=camera_config["view_matrix"],
        projection_matrix=camera_config["projection_matrix"],
    )
    detections = estimate_sortable_object_world_positions(
        rgb_image=camera_frame["rgb"],
        depth_image=camera_frame["depth"],
        camera_config=camera_config,
        object_center_z=TABLE_SURFACE_Z + CUBE_HALF_EXTENT,
    )

    for detection in detections:
        print(
            "[INFO] OpenCV detected "
            f"{detection['class_name']} shape={detection['shape']} "
            f"pixel={detection['center']} world={detection['world_position']}"
        )
    return detections


def _detections_by_class(detections: list[dict[str, object]]) -> dict[str, dict[str, object]]:
    by_class = {}
    for detection in detections:
        class_name = str(detection["class_name"])
        if class_name not in by_class:
            by_class[class_name] = detection
    return by_class


def _move_and_wait(robot_id: int, position: tuple[float, float, float], steps: int = MOVE_STEPS) -> None:
    move_end_effector_to_position(robot_id, position)
    step_simulation(steps)


def execute_pick_sequence(
    robot_id: int,
    gripper: SimulatedGripper,
    object_id: int,
    object_position: tuple[float, float, float] | None = None,
) -> None:
    object_x, object_y, object_z = object_position or _body_position(object_id)

    gripper.open()
    _move_and_wait(robot_id, (object_x, object_y, object_z + APPROACH_HEIGHT))
    _move_and_wait(robot_id, (object_x, object_y, object_z + GRASP_HEIGHT_OFFSET))

    gripper.grasp(object_id)
    _move_and_wait(robot_id, (object_x, object_y, object_z + APPROACH_HEIGHT))


def execute_place_sequence(
    robot_id: int,
    gripper: SimulatedGripper,
    target_zone_id: int,
) -> None:
    target_x, target_y, target_z = _body_position(target_zone_id)

    _move_and_wait(robot_id, (target_x, target_y, target_z + APPROACH_HEIGHT))
    _move_and_wait(robot_id, (target_x, target_y, target_z + GRASP_HEIGHT_OFFSET))
    gripper.release()
    step_simulation(120)
    _move_and_wait(robot_id, (target_x, target_y, target_z + APPROACH_HEIGHT))


def run_pick_and_place_workflow(robot_id: int, cube_id: int, target_zone_id: int) -> None:
    print("Starting pick-and-place workflow...")
    move_to_home(robot_id)
    step_simulation(240)
    perceived_cube_position = perceive_cube_position()
    if perceived_cube_position is None:
        perceived_cube_position = _body_position(cube_id)
        print(f"[INFO] Falling back to simulator cube pose: {perceived_cube_position}")

    gripper = SimulatedGripper(robot_id)
    execute_pick_sequence(robot_id, gripper, cube_id, perceived_cube_position)
    execute_place_sequence(robot_id, gripper, target_zone_id)
    move_to_home(robot_id)


def run_sorting_workflow(
    robot_id: int,
    object_bodies: dict[str, int],
    target_zones: dict[str, int],
    target_assignments: dict[str, str] | None = None,
) -> None:
    print("Starting OpenCV sorting workflow...")
    move_to_home(robot_id)
    step_simulation(240)

    gripper = SimulatedGripper(robot_id)
    detections = perceive_sortable_objects()
    detections_by_class = _detections_by_class(detections)
    assignments = target_assignments or {}

    for class_name in SORT_ORDER:
        object_id = object_bodies[class_name]
        target_class_name = assignments.get(class_name, class_name)
        target_zone_id = target_zones[target_class_name]
        detection = detections_by_class.get(class_name)

        if detection is None:
            object_position = _body_position(object_id)
            print(f"[WARN] No OpenCV detection for {class_name}; fallback pose={object_position}")
        else:
            object_position = detection["world_position"]

        print(f"[INFO] Sorting {class_name} into {target_class_name} target zone.")
        execute_pick_sequence(robot_id, gripper, object_id, object_position)
        execute_place_sequence(robot_id, gripper, target_zone_id)

    move_to_home(robot_id)
