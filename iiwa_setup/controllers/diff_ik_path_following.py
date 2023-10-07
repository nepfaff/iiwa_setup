from manipulation.scenarios import AddIiwaDifferentialIK
from pydrake.all import (
    Diagram,
    DiagramBuilder,
    DifferentialInverseKinematicsIntegrator,
    MultibodyPlant,
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

        differential_ik: DifferentialInverseKinematicsIntegrator = (
            AddIiwaDifferentialIK(
                builder,
                controller_plant,
                frame=controller_plant.GetFrameByName(control_frame_name),
            )
        )
        builder.ExportInput(
            differential_ik.GetInputPort("use_robot_state"), "reset_diff_ik"
        )
        builder.ExportInput(
            differential_ik.GetInputPort("robot_state"), "iiwa.state_estimated"
        )
        builder.Connect(
            pose_traj_source.get_output_port(),
            differential_ik.GetInputPort("X_WE_desired"),
        )

        builder.ExportOutput(differential_ik.get_output_port(), "iiwa.positions")

        builder.BuildInto(self)
