import pybullet as p


class SimulatedGripper:

    def __init__(self, robot_id: int, end_effector_link_index: int = 6) -> None:
        self.robot_id = robot_id
        self.end_effector_link_index = end_effector_link_index
        self.constraint_id: int | None = None

    def open(self) -> None:
        print("[INFO] gripper open")

    def close(self) -> None:
        print("[INFO] gripper close")

    def grasp(self, body_id: int) -> int:
        self.close()
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
