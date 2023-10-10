import numpy as np

from manipulation.scenarios import AddIiwaDifferentialIK
from pydrake.all import (
    AbstractValue,
    Context,
    Diagram,
    DiagramBuilder,
    DifferentialInverseKinematicsIntegrator,
    LeafSystem,
    MultibodyPlant,
    State,
)

from .trajectory_sources import PiecewisePoseWithTimingInformationSource


class DiffIKPathFollowingController(Diagram):
    """
    A controller for following a pose trajectory with differential inverse kinematics.
    """

    def __init__(self, controller_plant: MultibodyPlant, control_frame_name: str):
        """
        Args:
            controller_plant (MultibodyPlant): The robot controller plant for
            controlling the robot.
            control_frame_name (str): The poses in the pose trajectory are transforms
            from the world frame to the `control_frame_name` frame.
        """
        super().__init__()

        builder = DiagramBuilder()

        pose_traj_source: PiecewisePoseWithTimingInformationSource = (
            builder.AddNamedSystem(
                "pose_traj_source", PiecewisePoseWithTimingInformationSource()
            )
        )
        builder.ExportInput(pose_traj_source.get_input_port(), "pose_trajectory")

        self._differential_ik: DifferentialInverseKinematicsIntegrator = (
            AddIiwaDifferentialIK(
                builder,
                controller_plant,
                frame=controller_plant.GetFrameByName(control_frame_name),
            )
        )
        builder.Connect(
            pose_traj_source.get_output_port(),
            self._differential_ik.GetInputPort("X_WE_desired"),
        )

        builder.ExportOutput(self._differential_ik.get_output_port(), "iiwa.positions")

        builder.BuildInto(self)

    def reset_diff_ik(self, context: Context, joint_positions: np.ndarray) -> None:
        """Reset the diff IK state to allow integration from the new state.

        Args:
            context (Context): The simulator context that contains the
            DiffIKPathFollowingController system.
            joint_positions (np.ndarray): The joint positions that the state should be
            reset to.
        """
        self._differential_ik.get_mutable_parameters().set_nominal_joint_position(
            joint_positions
        )
        self._differential_ik.SetPositions(
            self._differential_ik.GetMyMutableContextFromRoot(context),
            joint_positions,
        )


class DiffIKReseter(LeafSystem):
    """
    A system for resetting the positions of a DiffIKPathFollowingController. This is
    required before using the controller.
    """

    def __init__(
        self,
        num_joint_positions: int,
        diff_ik_pose_path_follower: DiffIKPathFollowingController,
    ):
        """
        Args:
            num_joint_positions (int): The number of joint positions.
            diff_ik_pose_path_follower (DiffIKPathFollowingController): The controller
            that should be reset with this system.
        """
        super().__init__()

        self._diff_ik_pose_path_follower = diff_ik_pose_path_follower
        self._context = None

        self._reset_diff_ik_input_port = self.DeclareAbstractInputPort(
            "reset_diff_ik", AbstractValue.Make(False)
        )
        self._iiwa_position_measured_input_port = self.DeclareVectorInputPort(
            "iiwa.position_measured", num_joint_positions
        )

        self.DeclarePerStepUnrestrictedUpdateEvent(self._reset_diff_ik)

    def set_context(self, context: Context) -> None:
        """
        Sets the simulator context. NOTE: This must be called before starting the
        simulation.
        """
        self._context = context

    def _reset_diff_ik(self, context: Context, state: State) -> None:
        if self._reset_diff_ik_input_port.Eval(context):
            assert (
                self._context is not None
            ), "Need to set the context using 'set_context' before simulating!"

            current_positions = self._iiwa_position_measured_input_port.Eval(context)
            self._diff_ik_pose_path_follower.reset_diff_ik(
                self._context, current_positions
            )
