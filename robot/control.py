from typing import Iterable, Sequence


def move_to_home(robot_id: int, home_joint_positions: Sequence[float] | None = None) -> None:
    """Move the robot to a predefined home pose (placeholder)."""
    _ = robot_id
    _ = home_joint_positions
    print("[TODO] move_to_home: implement joint-space homing logic.")


def move_end_effector_to_position(
    robot_id: int,
    target_position: Sequence[float],
    target_orientation: Sequence[float] | None = None,
) -> None:
    """Move end effector to target Cartesian position (placeholder)."""
    _ = robot_id
    _ = target_position
    _ = target_orientation
    print("[TODO] move_end_effector_to_position: implement IK + joint control.")


def open_gripper(robot_id: int, gripper_joint_indices: Iterable[int] | None = None) -> None:
    """Open gripper fingers (placeholder)."""
    _ = robot_id
    _ = gripper_joint_indices
    print("[TODO] open_gripper: implement gripper open command.")


def close_gripper(robot_id: int, gripper_joint_indices: Iterable[int] | None = None) -> None:
    """Close gripper fingers (placeholder)."""
    _ = robot_id
    _ = gripper_joint_indices
    print("[TODO] close_gripper: implement gripper close command.")
