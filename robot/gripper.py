import pybullet as p


OPEN_FINGER_ANGLE = 0.30
CLOSED_FINGER_ANGLE = 0.0
FINGER_FORCE = 40.0
FINGER_TIP_FORCE = 20.0
GRIPPER_SETTLE_STEPS = 80
FINGER_JOINT_TARGET_DIRECTIONS = {
    "base_left_finger_joint": -1.0,
    "left_base_tip_joint": 0.0,
    "base_right_finger_joint": 1.0,
    "right_base_tip_joint": 0.0,
}


class KukaWSG50Gripper:
    """Drive the WSG50 finger joints on the built-in KUKA gripper model."""

    def __init__(
        self,
        robot_id: int,
        end_effector_link_index: int = 7,
        assisted_grasp: bool = True,
    ) -> None:
        self.robot_id = robot_id
        self.end_effector_link_index = end_effector_link_index
        self.assisted_grasp = assisted_grasp
        self.constraint_id: int | None = None
        self.joint_name_to_index = {
            p.getJointInfo(robot_id, joint_index)[1].decode("utf-8"): joint_index
            for joint_index in range(p.getNumJoints(robot_id))
        }

    def _available_finger_targets(self, angle: float) -> tuple[list[int], list[float], list[float]]:
        joint_indices = []
        target_positions = []
        forces = []

        for joint_name, direction in FINGER_JOINT_TARGET_DIRECTIONS.items():
            joint_index = self.joint_name_to_index.get(joint_name)

            if joint_index is None:
                continue

            joint_indices.append(joint_index)
            target_positions.append(direction * angle)
            forces.append(FINGER_TIP_FORCE if direction == 0.0 else FINGER_FORCE)

        return joint_indices, target_positions, forces

    def _command_fingers(self, angle: float) -> None:
        joint_indices, target_positions, forces = self._available_finger_targets(angle)

        if not joint_indices:
            print("[WARN] No WSG50 gripper joints were found on this robot.")
            return

        p.setJointMotorControlArray(
            bodyUniqueId=self.robot_id,
            jointIndices=joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=target_positions,
            forces=forces,
            positionGains=[0.30] * len(joint_indices),
            velocityGains=[1.0] * len(joint_indices),
        )

    def _settle(self, steps: int = GRIPPER_SETTLE_STEPS) -> None:
        for _ in range(steps):
            p.stepSimulation()

    def open(self) -> None:
        self._command_fingers(OPEN_FINGER_ANGLE)
        self._settle(40)
        print("[INFO] WSG50 gripper open")

    def close(self) -> None:
        self._command_fingers(CLOSED_FINGER_ANGLE)
        self._settle()
        print("[INFO] WSG50 gripper close")

    def grasp(self, body_id: int) -> int:
        self.close()

        if not self.assisted_grasp:
            return -1

        wrist_position, wrist_orientation = p.getLinkState(
            self.robot_id,
            self.end_effector_link_index,
        )[:2]
        body_position, body_orientation = p.getBasePositionAndOrientation(body_id)

        inverse_wrist_position, inverse_wrist_orientation = p.invertTransform(
            wrist_position,
            wrist_orientation,
        )
        parent_frame_position, parent_frame_orientation = p.multiplyTransforms(
            inverse_wrist_position,
            inverse_wrist_orientation,
            body_position,
            body_orientation,
        )

        self.constraint_id = p.createConstraint(
            parentBodyUniqueId=self.robot_id,
            parentLinkIndex=self.end_effector_link_index,
            childBodyUniqueId=body_id,
            childLinkIndex=-1,
            jointType=p.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=parent_frame_position,
            childFramePosition=[0, 0, 0],
            parentFrameOrientation=parent_frame_orientation,
            childFrameOrientation=[0, 0, 0, 1],
        )
        p.changeConstraint(self.constraint_id, maxForce=500.0)
        return self.constraint_id

    def release(self) -> None:
        if self.constraint_id is not None:
            p.removeConstraint(self.constraint_id)
            self.constraint_id = None
        self.open()


SimulatedGripper = KukaWSG50Gripper
