import logging

from typing import Optional, Union

import numpy as np

from pydrake.all import (
    CompositeTrajectory,
    GcsTrajectoryOptimization,
    HPolyhedron,
    MultibodyPlant,
    Point,
)


def plan_unconstrained_gcs_path_start_to_goal(
    plant: MultibodyPlant,
    q_goal: np.ndarray,
    q_start: np.ndarray,
    velocity_limits: Optional[np.ndarray] = None,
) -> Union[CompositeTrajectory, None]:
    """Plan an unconstrained trajectory from a start to a goal in joint space using GCS.

    Args:
        plant (MultibodyPlant): The plant of the robot to plan the trajectory for.
        q_goal (np.ndarray): The starting joint positions.
        q_start (np.ndarray): The goal joint positions
        velocity_limits (Optional[np.ndarray]): The optional velocity limits.

    Returns:
        Union[CompositeTrajectory, None]: The trajectory if success and None if failure.
    """
    num_positions = plant.num_positions()
    assert num_positions == len(q_goal) == len(q_start)

    gcs = GcsTrajectoryOptimization(num_positions)

    # Ignore obstacles
    workspace = gcs.AddRegions(
        regions=[
            HPolyhedron.MakeBox(
                plant.GetPositionLowerLimits(), plant.GetPositionUpperLimits()
            )
        ],
        order=5,
        h_min=1,
        h_max=60,
    )
    # Set non-zero h_min for start and goal to enforce zero velocity
    start = gcs.AddRegions([Point(q_start)], order=1, h_min=0.1)
    goal = gcs.AddRegions([Point(q_goal)], order=1, h_min=0.1)
    gcs.AddEdges(start, workspace)
    gcs.AddEdges(workspace, goal)

    if velocity_limits is not None:
        gcs.AddVelocityBounds(-velocity_limits, velocity_limits)

    gcs.AddTimeCost()
    gcs.AddPathLengthCost()

    logging.info(f"Planning unconstrained GCS path from {q_start} to {q_goal}.")
    traj, result = gcs.SolvePath(start, goal)
    if not result.is_success():
        logging.error(
            f"Failed to plan unconstrained GCS path from {q_start} to {q_goal}."
        )
        return None
    return traj
