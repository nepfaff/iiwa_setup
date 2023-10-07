import logging

from typing import Optional

import numpy as np

from pydrake.all import (
    InverseKinematics,
    MultibodyPlant,
    RigidTransform,
    RotationMatrix,
    Solve,
)


def solve_global_inverse_kinematics(
    plant: MultibodyPlant,
    X_G: RigidTransform,
    initial_guess: np.ndarray,
    position_tolerance: float,
    orientation_tolerance: float,
    gripper_frame_name: str = "body",
) -> Optional[np.ndarray]:
    """Computes global IK.

    Args:
        plant (MultibodyPlant): The robot control plant.
        X_G (RigidTransform): Gripper pose to compute the joint angles for.
        initial_guess (np.ndarray): The initial guess to use of shape (N,) where N are
        the number of joint positions.
        position_tolerance (float): The position tolerance to use for the global IK
        optimization problem.
        orientation_tolerance (float): The orientation tolerance to use for the global
        IK optimization problem.
        gripper_frame_name (str): The name of the gripper frame.

    Returns:
        Optional[np.ndarray]: Joint positions corresponding to X_G. Returns None if no
        IK solution could be found.
    """
    ik = InverseKinematics(plant)
    q_variables = ik.q()

    gripper_frame = plant.GetFrameByName(gripper_frame_name)

    # Position constraint
    p_G_ref = X_G.translation()
    ik.AddPositionConstraint(
        frameB=gripper_frame,
        p_BQ=np.zeros(3),
        frameA=plant.world_frame(),
        p_AQ_lower=p_G_ref - position_tolerance,
        p_AQ_upper=p_G_ref + position_tolerance,
    )

    # Orientation constraint
    R_G_ref = X_G.rotation()
    ik.AddOrientationConstraint(
        frameAbar=plant.world_frame(),
        R_AbarA=R_G_ref,
        frameBbar=gripper_frame,
        R_BbarB=RotationMatrix(),
        theta_bound=orientation_tolerance,
    )

    prog = ik.prog()
    prog.SetInitialGuess(q_variables, initial_guess)

    result = Solve(prog)
    if not result.is_success():
        logging.error(f"Failed to solve global IK for gripper pose {X_G}.")
        return None
    q_sol = result.GetSolution(q_variables)
    return q_sol
