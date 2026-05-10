import time
from typing import Iterable, Sequence

import pybullet as p


DEFAULT_HOME_JOINT_POSITIONS = [0.0, -0.45, 0.0, 1.25, 0.0, -0.85, 0.0]
ARM_JOINT_NAMES = {f"J{joint_index}" for joint_index in range(7)}
GRIPPER_BASE_LINK_NAME = "base_link"


def get_controllable_joints(robot_id: int) -> list[int]:
    """Return revolute/prismatic joints that can be commanded."""
    return [
        joint_index
        for joint_index in range(p.getNumJoints(robot_id))
        if p.getJointInfo(robot_id, joint_index)[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)
    ]


def get_arm_joints(robot_id: int) -> list[int]:
    """Return only the seven KUKA arm joints, excluding gripper joints."""
    arm_joint_indices = []

    for joint_index in range(p.getNumJoints(robot_id)):
        joint_info = p.getJointInfo(robot_id, joint_index)
        joint_name = joint_info[1].decode("utf-8")

        if joint_name in ARM_JOINT_NAMES:
            arm_joint_indices.append(joint_index)

    if arm_joint_indices:
        return sorted(arm_joint_indices)

    return get_controllable_joints(robot_id)[:7]


def get_end_effector_link_index(robot_id: int) -> int:
    """Return the gripper base link when present, otherwise the arm wrist."""
    for joint_index in range(p.getNumJoints(robot_id)):
        link_name = p.getJointInfo(robot_id, joint_index)[12].decode("utf-8")

        if link_name == GRIPPER_BASE_LINK_NAME:
            return joint_index

    return get_arm_joints(robot_id)[-1]


def step_simulation(steps: int = 240, real_time: bool = True) -> None:
    for _ in range(steps):
        p.stepSimulation()
        if real_time and p.getConnectionInfo().get("connectionMethod") == p.GUI:
            time.sleep(1.0 / 240.0)


def move_to_home(robot_id: int, home_joint_positions: Sequence[float] | None = None) -> None:
    """Command the robot to a stable home pose."""
    joint_indices = get_arm_joints(robot_id)
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
    joint_indices = get_arm_joints(robot_id)
    end_effector_link_index = get_end_effector_link_index(robot_id)
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
    """Open the WSG50 fingers when the loaded robot has gripper joints."""
    default_targets = {8: -0.30, 10: 0.0, 11: 0.30, 13: 0.0}
    joint_indices = [
        joint_index
        for joint_index in list(gripper_joint_indices or default_targets)
        if joint_index < p.getNumJoints(robot_id)
    ]

    if not joint_indices:
        return

    p.setJointMotorControlArray(
        bodyUniqueId=robot_id,
        jointIndices=joint_indices,
        controlMode=p.POSITION_CONTROL,
        targetPositions=[default_targets.get(joint_index, 0.0) for joint_index in joint_indices],
        forces=[40.0] * len(joint_indices),
    )


def close_gripper(robot_id: int, gripper_joint_indices: Iterable[int] | None = None) -> None:
    """Close the WSG50 fingers when the loaded robot has gripper joints."""
    joint_indices = [
        joint_index
        for joint_index in list(gripper_joint_indices or [8, 10, 11, 13])
        if joint_index < p.getNumJoints(robot_id)
    ]

    if not joint_indices:
        return

    p.setJointMotorControlArray(
        bodyUniqueId=robot_id,
        jointIndices=joint_indices,
        controlMode=p.POSITION_CONTROL,
        targetPositions=[0.0] * len(joint_indices),
        forces=[40.0] * len(joint_indices),
    )
