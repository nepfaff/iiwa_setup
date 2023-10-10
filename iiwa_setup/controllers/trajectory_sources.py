import numpy as np

from pydrake.all import AbstractValue, BasicVector, Context, LeafSystem, RigidTransform

from iiwa_setup.dataclasses import (
    PiecewisePoseWithTimingInformation,
    TrajectoryWithTimingInformation,
)


class TrajectoryWithTimingInformationSource(LeafSystem):
    """
    A trajectory source that outputs trajectory values while respecting the trajectory
    start time.
    """

    def __init__(self, trajectory_size: int):
        """
        Args:
            trajectory_size (int): The number of dimensions of the trajectory values.
            For robot joint trajectories, this should be equal to the number of joint
            positions.
        """
        super().__init__()

        # The current_cmd that should be passed to output when the current trajectory is invalid
        self._current_cmd_input_port = self.DeclareVectorInputPort(
            "current_cmd", trajectory_size
        )
        self._trajectory_input_port = self.DeclareAbstractInputPort(
            "trajectory", AbstractValue.Make(TrajectoryWithTimingInformation())
        )
        self.DeclareVectorOutputPort(
            "output", trajectory_size, self._calc_trajectory_value
        )

    def _calc_trajectory_value(self, context: Context, output: BasicVector) -> None:
        current_traj: TrajectoryWithTimingInformation = (
            self._trajectory_input_port.Eval(context)
        )
        traj_time = context.get_time() - current_traj.start_time_s
        if np.isnan(traj_time):
            traj_value = self._current_cmd_input_port.Eval(context)
        else:
            traj_value = current_traj.trajectory.value(traj_time).ravel()
        output.SetFromVector(traj_value)


class PiecewisePoseWithTimingInformationSource(LeafSystem):
    """
    A pose trajectory source that outputs trajectory values while respecting the
    trajectory start time.
    """

    def __init__(self):
        super().__init__()

        self._trajectory_input_port = self.DeclareAbstractInputPort(
            "pose_trajectory", AbstractValue.Make(PiecewisePoseWithTimingInformation)
        )
        self.DeclareAbstractOutputPort(
            "pose",
            lambda: AbstractValue.Make(RigidTransform()),
            self._calc_trajectory_value,
        )

    def _calc_trajectory_value(self, context: Context, output: AbstractValue) -> None:
        current_traj: PiecewisePoseWithTimingInformation = (
            self._trajectory_input_port.Eval(context)
        )
        if np.isnan(current_traj.start_time_s):
            return
        traj_time = context.get_time() - current_traj.start_time_s
        traj_value = current_traj.trajectory.GetPose(traj_time)
        output.set_value(traj_value)
