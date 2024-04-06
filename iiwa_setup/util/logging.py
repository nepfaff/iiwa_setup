import logging

from pydrake.all import Context, DiscreteValues, LeafSystem, RigidBodyFrame


class NoDrakeDifferentialIKFilter(logging.Filter):
    def filter(self, record):
        return not record.getMessage().startswith("Differential IK")


class BodyPoseLogger(LeafSystem):
    """System for logging a body's pose."""

    def __init__(self, body_frame: RigidBodyFrame, period_s: float):
        """
        Args:
            body_frame (RigidBodyFrame): The body frame of the body.
            period_s (float): The logging period.
        """
        super().__init__()

        self._body_frame = body_frame
        self._plant_context = None

        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=period_s, offset_sec=0.0, update=self._log_pose
        )

    def _log_pose(self, context: Context, discrete_values: DiscreteValues) -> None:
        assert (
            self._plant_context is not None
        ), "The plant context must be set before the simulation is started!"

        X_WB = self._body_frame.CalcPoseInWorld(self._plant_context)
        logging.info(
            f"Translation: {X_WB.translation()}, Rotation: {X_WB.rotation().ToRollPitchYaw()}"
        )

    def set_plant_context(self, context: Context) -> None:
        """
        Sets the plant context. NOTE: The context must be set before simulation is
        started. This must be the plant that contains the body that is associated to
        `body_frame`.
        """
        self._plant_context = context
