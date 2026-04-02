from typing import Dict, Tuple

import pybullet as p


def load_robot(
    base_position: Tuple[float, float, float] = (0.0, 0.0, 0.0),
    use_fixed_base: bool = True,
) -> Dict[str, object]:
    """Load a built-in KUKA iiwa arm and return useful metadata."""
    robot_id = p.loadURDF(
        "kuka_iiwa/model.urdf",
        basePosition=base_position,
        useFixedBase=use_fixed_base,
    )

    joint_count = p.getNumJoints(robot_id)
    controllable_joint_indices = [
        joint_index
        for joint_index in range(joint_count)
        if p.getJointInfo(robot_id, joint_index)[2] in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC)
    ]

    return {
        "robot_id": robot_id,
        "joint_count": joint_count,
        "controllable_joint_indices": controllable_joint_indices,
        "end_effector_link_index": 6,
        "name": "kuka_iiwa",
    }
