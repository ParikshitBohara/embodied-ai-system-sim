from typing import Dict, Tuple

import pybullet as p


KUKA_WITH_GRIPPER_MODEL = "kuka_iiwa/kuka_with_gripper.sdf"
ARM_JOINT_NAMES = {f"J{joint_index}" for joint_index in range(7)}
GRIPPER_JOINT_NAMES = (
    "base_left_finger_joint",
    "left_base_tip_joint",
    "base_right_finger_joint",
    "right_base_tip_joint",
)


def load_robot(
    base_position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    use_fixed_base: bool = True,
) -> Dict[str, object]:
    """Load a built-in KUKA iiwa arm with a WSG50 gripper."""
    _ = use_fixed_base

    robot_id = p.loadSDF(KUKA_WITH_GRIPPER_MODEL)[0]
    p.resetBasePositionAndOrientation(
        robot_id,
        base_position,
        [0, 0, 0, 1],
    )

    joint_count = p.getNumJoints(robot_id)
    joint_name_to_index = {
        p.getJointInfo(robot_id, joint_index)[1].decode("utf-8"): joint_index
        for joint_index in range(joint_count)
    }
    arm_joint_indices = [
        joint_name_to_index[joint_name]
        for joint_name in sorted(ARM_JOINT_NAMES, key=lambda name: int(name[1:]))
        if joint_name in joint_name_to_index
    ]
    gripper_joint_indices = [
        joint_name_to_index[joint_name]
        for joint_name in GRIPPER_JOINT_NAMES
        if joint_name in joint_name_to_index
    ]
    controllable_joint_indices = [
        joint_index
        for joint_index in range(joint_count)
        if p.getJointInfo(robot_id, joint_index)[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)
    ]

    return {
        "robot_id": robot_id,
        "joint_count": joint_count,
        "arm_joint_indices": arm_joint_indices,
        "gripper_joint_indices": gripper_joint_indices,
        "controllable_joint_indices": controllable_joint_indices,
        "end_effector_link_index": joint_name_to_index.get("gripper_to_arm", 6),
        "name": "kuka_iiwa_with_wsg50",
    }
