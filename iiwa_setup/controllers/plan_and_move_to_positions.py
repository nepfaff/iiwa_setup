import logging

from typing import Union

import numpy as np

from pydrake.all import (
    BasicVector,
    Context,
    LeafSystem,
    MultibodyPlant,
    PiecewisePolynomial,
    Trajectory,
)

from iiwa_setup.motion_planning import (
    plan_unconstrained_gcs_path_start_to_goal,
    reparameterize_with_toppra,
)


class PlanAndMoveToPositionsUnconstrainedController(LeafSystem):
    """
    A system that plans an unconstrained (not considering collisions with the
    environment) trajectory from the current positions to the goal positions and
    executes the trajectory.
    """

    def __init__(
        self,
        controller_plant: MultibodyPlant,
        goal_positions: np.ndarray,
        velocity_limits: np.ndarray,
        acceleration_limits: np.ndarray,
    ):
        super().__init__()

        self._controller_plant = controller_plant
        self._goal_positions = goal_positions
        self._velocity_limits = velocity_limits
        self._acceleration_limits = acceleration_limits
        self._position_trajectory: Union[Trajectory, None] = None
        self._is_finished = False

        num_positions = len(goal_positions)
        self._positions_measured_input_port = self.DeclareVectorInputPort(
            "position_measured", num_positions
        )
        self.DeclareVectorOutputPort(
            "positions_commanded", num_positions, self._calc_trajectory_value
        )

    def _calc_trajectory_value(self, context: Context, output: BasicVector) -> None:
        if self._position_trajectory is None:
            current_positions: np.ndarray = self._positions_measured_input_port.Eval(
                context
            )
            if np.allclose(current_positions, self._goal_positions):
                logging.info("Already at the desired positions.")
                self._position_trajectory = PiecewisePolynomial.ZeroOrderHold(
                    breaks=[0.0, 1.0], samples=np.zeros((7, 2))
                )
                self._is_finished = True
            else:
                traj = plan_unconstrained_gcs_path_start_to_goal(
                    plant=self._controller_plant,
                    q_start=current_positions,
                    q_goal=self._goal_positions,
                )
                if traj is None:
                    logging.error("Failed to find a path to the goal positions.")
                    exit(1)
                toppra_traj = reparameterize_with_toppra(
                    trajectory=traj,
                    plant=self._controller_plant,
                    velocity_limits=self._velocity_limits,
                    acceleration_limits=self._acceleration_limits,
                )
                self._position_trajectory = toppra_traj

        curent_time = context.get_time()
        traj_value = self._position_trajectory.value(curent_time).ravel()
        output.SetFromVector(traj_value)

        if curent_time >= self._position_trajectory.end_time():
            self._is_finished = True

    def is_finished(self) -> bool:
        return self._is_finished
