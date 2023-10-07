import numpy as np

from pydrake.all import MultibodyPlant, PathParameterizedTrajectory, Toppra, Trajectory


def reparameterize_with_toppra(
    trajectory: Trajectory,
    plant: MultibodyPlant,
    velocity_limits: np.ndarray,
    acceleration_limits: np.ndarray,
    num_grid_points: int = 1000,
) -> PathParameterizedTrajectory:
    """Reparameterize a trajectory/ path with Toppra.

    Args:
        trajectory (Trajectory): The trajectory on which the TOPPRA problem will be
        solved.
        plant (MultibodyPlant): The robot that will follow the solved trajectory. Used
        for enforcing torque and frame specific constraints.
        velocity_limits (np.ndarray): The velocity limits of shape (N,) where N is the
        number of robot joint joints.
        acceleration_limits (np.ndarray): The acceleration limits of shape (N,) where N
        is the number of robot joint joints.
        num_grid_points (int, optional): The number of uniform points along the path to
        discretize the problem and enforce constraints at.

    Returns:
        PathParameterizedTrajectory: The reparameterized trajectory.
    """
    toppra = Toppra(
        path=trajectory,
        plant=plant,
        gridpoints=np.linspace(
            trajectory.start_time(), trajectory.end_time(), num_grid_points
        ),
    )
    toppra.AddJointVelocityLimit(-velocity_limits, velocity_limits)
    toppra.AddJointAccelerationLimit(-acceleration_limits, acceleration_limits)
    time_trajectory = toppra.SolvePathParameterization()
    return PathParameterizedTrajectory(trajectory, time_trajectory)
