from dataclasses import dataclass

import numpy as np

from pydrake.all import PiecewisePolynomial, PiecewisePose, Trajectory


@dataclass
class TrajectoryWithTimingInformation:
    """
    Contains a Drake Trajectory and an associated start time which is relative to
    the simulation start time.
    """

    trajectory: Trajectory = PiecewisePolynomial()
    start_time_s: float = np.nan


@dataclass
class PiecewisePoseWithTimingInformation:
    """
    Contains a Drake PiecewisePose trajectory and an associated start time which is
    relative to the simulation start time.
    """

    trajectory: PiecewisePose = PiecewisePose()
    start_time_s: float = np.nan
