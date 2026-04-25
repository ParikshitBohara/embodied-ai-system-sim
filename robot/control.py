import time
from typing import Iterable, Sequence

import pybullet as p


DEFAULT_HOME_JOINT_POSITIONS = [0.0, -0.45, 0.0, 1.25, 0.0, -0.85, 0.0]


def get_controllable_joints(robot_id: int) -> list[int]:
    """Return revolute/prismatic joints that can be commanded."""
    return [
        joint_index
        for joint_index in range(p.getNumJoints(robot_id))
        if p.getJointInfo(robot_id, joint_index)[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)
    ]


def step_simulation(steps: int = 240, real_time: bool = True) -> None:
    for _ in range(steps):
        p.stepSimulation()
        if real_time and p.getConnectionInfo().get("connectionMethod") == p.GUI:
            time.sleep(1.0 / 240.0)


def move_to_home(robot_id: int, home_joint_positions: Sequence[float] | None = None) -> None:
    """Command the robot to a stable home pose."""
    joint_indices = get_controllable_joints(robot_id)
    target_positions = list(home_joint_positions or DEFAULT_HOME_JOINT_POSITIONS)

    if len(target_positions) != len(joint_indices):
        raise ValueError(
            f"Expected {len(joint_indices)} home joint positions, got {len(target_positions)}."
        )

    p.setJointMotorControlArray(
        bodyUniqueId=robot_id,
        jointIndices=joint_indices,
        controlMode=p.POSITION_CONTROL,
        targetPositions=target_positions,
        forces=[300.0] * len(joint_indices),
        positionGains=[0.04] * len(joint_indices),
        velocityGains=[1.0] * len(joint_indices),
    )


def move_end_effector_to_position(
    robot_id: int,
    target_position: Sequence[float],
    target_orientation: Sequence[float] | None = None,
) -> None:
    """Command the KUKA end effector to a Cartesian target using inverse kinematics."""
    joint_indices = get_controllable_joints(robot_id)
    end_effector_link_index = joint_indices[-1]
    ik_kwargs = {
        "bodyUniqueId": robot_id,
        "endEffectorLinkIndex": end_effector_link_index,
        "targetPosition": target_position,
        "maxNumIterations": 200,
        "residualThreshold": 1e-4,
    }
    if target_orientation is not None:
        ik_kwargs["targetOrientation"] = target_orientation

    joint_positions = p.calculateInverseKinematics(**ik_kwargs)
    joint_targets = list(joint_positions[: len(joint_indices)])

    p.setJointMotorControlArray(
        bodyUniqueId=robot_id,
        jointIndices=joint_indices,
        controlMode=p.POSITION_CONTROL,
        targetPositions=joint_targets,
        forces=[300.0] * len(joint_indices),
        positionGains=[0.05] * len(joint_indices),
        velocityGains=[1.0] * len(joint_indices),
    )


def open_gripper(robot_id: int, gripper_joint_indices: Iterable[int] | None = None) -> None:
    """Placeholder for compatibility: the bundled KUKA model has no gripper."""
    _ = robot_id
    _ = gripper_joint_indices
    print("[INFO] open_gripper: no gripper joints on the built-in KUKA model.")


def close_gripper(robot_id: int, gripper_joint_indices: Iterable[int] | None = None) -> None:
    """Placeholder for compatibility: the bundled KUKA model has no gripper."""
    _ = robot_id
    _ = gripper_joint_indices
    print("[INFO] close_gripper: no gripper joints on the built-in KUKA model.")
