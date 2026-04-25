import pybullet as p

from robot.control import (
    close_gripper,
    move_end_effector_to_position,
    move_to_home,
    open_gripper,
    step_simulation,
)


APPROACH_HEIGHT = 0.25
GRASP_HEIGHT_OFFSET = 0.12
MOVE_STEPS = 180


def _body_position(body_id: int) -> tuple[float, float, float]:
    position, _ = p.getBasePositionAndOrientation(body_id)
    return position


def _move_and_wait(robot_id: int, position: tuple[float, float, float], steps: int = MOVE_STEPS) -> None:
    move_end_effector_to_position(robot_id, position)
    step_simulation(steps)


def _create_grasp_constraint(robot_id: int, cube_id: int) -> int:
    end_effector_state = p.getLinkState(robot_id, 6)
    wrist_position = end_effector_state[0]
    wrist_orientation = end_effector_state[1]
    cube_position, cube_orientation = p.getBasePositionAndOrientation(cube_id)

    inverse_wrist_position, inverse_wrist_orientation = p.invertTransform(
        wrist_position,
        wrist_orientation,
    )
    parent_frame_position, parent_frame_orientation = p.multiplyTransforms(
        inverse_wrist_position,
        inverse_wrist_orientation,
        cube_position,
        cube_orientation,
    )

    constraint_id = p.createConstraint(
        parentBodyUniqueId=robot_id,
        parentLinkIndex=6,
        childBodyUniqueId=cube_id,
        childLinkIndex=-1,
        jointType=p.JOINT_FIXED,
        jointAxis=[0, 0, 0],
        parentFramePosition=parent_frame_position,
        childFramePosition=[0, 0, 0],
        parentFrameOrientation=parent_frame_orientation,
        childFrameOrientation=[0, 0, 0, 1],
    )
    p.changeConstraint(constraint_id, maxForce=500.0)
    return constraint_id


def execute_pick_sequence(robot_id: int, cube_id: int) -> int:
    """Approach the cube, attach it to the wrist, and lift it."""
    cube_x, cube_y, cube_z = _body_position(cube_id)

    open_gripper(robot_id)
    _move_and_wait(robot_id, (cube_x, cube_y, cube_z + APPROACH_HEIGHT))
    _move_and_wait(robot_id, (cube_x, cube_y, cube_z + GRASP_HEIGHT_OFFSET))
    close_gripper(robot_id)

    constraint_id = _create_grasp_constraint(robot_id, cube_id)
    _move_and_wait(robot_id, (cube_x, cube_y, cube_z + APPROACH_HEIGHT))
    return constraint_id


def execute_place_sequence(robot_id: int, target_zone_id: int, grasp_constraint_id: int) -> None:
    """Move above the target marker and release the simulated grasp."""
    target_x, target_y, target_z = _body_position(target_zone_id)

    _move_and_wait(robot_id, (target_x, target_y, target_z + APPROACH_HEIGHT))
    _move_and_wait(robot_id, (target_x, target_y, target_z + GRASP_HEIGHT_OFFSET))
    p.removeConstraint(grasp_constraint_id)
    open_gripper(robot_id)
    step_simulation(120)
    _move_and_wait(robot_id, (target_x, target_y, target_z + APPROACH_HEIGHT))


def run_pick_and_place_workflow(robot_id: int, cube_id: int, target_zone_id: int) -> None:
    """Run a simple pick-and-place workflow in simulation."""
    print("Starting pick-and-place workflow...")
    move_to_home(robot_id)
    step_simulation(240)
    grasp_constraint_id = execute_pick_sequence(robot_id, cube_id)
    execute_place_sequence(robot_id, target_zone_id, grasp_constraint_id)
    move_to_home(robot_id)
