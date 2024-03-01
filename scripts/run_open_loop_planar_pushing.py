import argparse
import logging

import numpy as np

from manipulation.station import LoadScenario
from pydrake.all import (
    ApplySimulatorConfig,
    DiagramBuilder,
    MeshcatVisualizer,
    PiecewisePose,
    RigidTransform,
    RollPitchYaw,
    Simulator,
)

from iiwa_setup.controllers import OpenLoopPlanarPushingController
from iiwa_setup.iiwa import IiwaHardwareStationDiagram
from iiwa_setup.util import NoDrakeDifferentialIKFilter


def main(
    scenario_str: str,
    use_hardware: bool,
    html_path: str,
    pushing_start_pose: RigidTransform,
    pushing_pose_trajectory: PiecewisePose,
    move_to_start_velocity_limits: np.ndarray,
    move_to_start_acceleration_limits: np.ndarray,
) -> None:
    builder = DiagramBuilder()

    scenario = LoadScenario(data=scenario_str)
    station: IiwaHardwareStationDiagram = builder.AddNamedSystem(
        "station",
        IiwaHardwareStationDiagram(
            scenario=scenario, has_wsg=False, use_hardware=use_hardware
        ),
    )

    controller_plant = station.get_iiwa_controller_plant()
    controller: OpenLoopPlanarPushingController = builder.AddNamedSystem(
        "open_loop_planar_pushing_controller",
        OpenLoopPlanarPushingController(
            controller_plant=controller_plant,
            num_joint_positions=7,
            pushing_start_pose=pushing_start_pose,
            pushing_pose_trajectory=pushing_pose_trajectory,
            gripper_frame_name="iiwa_link_7",
            initial_delay_s=1.0,
            wait_push_delay_s=1.0,
            move_to_start_velocity_limits=move_to_start_velocity_limits,
            move_to_start_acceleration_limits=move_to_start_acceleration_limits,
        ),
    )
    builder.Connect(
        station.GetOutputPort("iiwa.position_measured"),
        controller.GetInputPort("iiwa.position_measured"),
    )
    builder.Connect(
        controller.GetOutputPort("iiwa.position"),
        station.GetInputPort("iiwa.position"),
    )

    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, station.GetOutputPort("query_object"), station.internal_meshcat
    )

    diagram = builder.Build()
    simulator = Simulator(diagram)
    ApplySimulatorConfig(scenario.simulator_config, simulator)
    simulator.set_target_realtime_rate(1.0)

    context = simulator.get_context()
    controller.set_context(context)

    visualizer.StartRecording()
    station.internal_meshcat.AddButton("Stop Simulation")
    while (
        station.internal_meshcat.GetButtonClicks("Stop Simulation") < 1
        and not controller.is_finished()
    ):
        simulator.AdvanceTo(context.get_time() + 0.1)
    station.internal_meshcat.DeleteButton("Stop Simulation")
    visualizer.StopRecording()
    visualizer.PublishRecording()

    if html_path is not None:
        html = station.internal_meshcat.StaticHtml()
        with open(html_path, "w") as f:
            f.write(html)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--use_hardware",
        action="store_true",
        help="Whether to use real world hardware.",
    )
    parser.add_argument(
        "--html_path",
        type=str,
        default=None,
        help="The path to save the simulation recording Meshcat HTML to. No HTML is "
        + "saved if no path is provided.",
    )
    parser.add_argument(
        "--log_level",
        type=str,
        default="INFO",
        choices=["CRITICAL", "ERROR", "WARNING", "INFO", "DEBUG"],
        help="Log level.",
    )

    args = parser.parse_args()

    logging.basicConfig(level=args.log_level)
    logging.getLogger("drake").addFilter(NoDrakeDifferentialIKFilter())

    scenario_str = f"""
    directives:
    - add_directives:
        file: package://iiwa_setup/iiwa7_with_planar_pusher.dmd.yaml
    plant_config:
        # Cannot use bigger timesteps on the real robot
        time_step: 0.001
        contact_model: "hydroelastic"
        discrete_contact_approximation: "sap"
    model_drivers:
        iiwa: !IiwaDriver
            lcm_bus: "default"
            control_mode: position_only
    lcm_buses:
        default:
            lcm_url: ""
    """

    poses = [
        RigidTransform(RollPitchYaw(np.pi, 0.0, -np.pi), [0.4, 0.0, 0.4]),
        RigidTransform(RollPitchYaw(np.pi, 0.0, -np.pi), [0.5, 0.0, 0.4]),
        RigidTransform(RollPitchYaw(np.pi, 0.0, -np.pi), [0.6, 0.0, 0.4]),
        RigidTransform(RollPitchYaw(np.pi, 0.0, -np.pi), [0.7, 0.0, 0.4]),
        # RigidTransform(RollPitchYaw(np.pi, 0.0, -np.pi), [0.75, 0.0, 0.4]),
    ]

    # Rotate the gripper by 90 degrees around yaw in the gripper frame
    X_GG1 = RigidTransform(RollPitchYaw(0.0, 0.0, np.pi / 2.0), np.zeros(3))
    poses = [pose @ X_GG1 for pose in poses]

    pushing_pose_traj = PiecewisePose.MakeLinear(
        times=np.arange(len(poses)),
        poses=poses,
    )

    main(
        scenario_str=scenario_str,
        use_hardware=args.use_hardware,
        html_path=args.html_path,
        pushing_start_pose=poses[0],
        pushing_pose_trajectory=pushing_pose_traj,
        move_to_start_velocity_limits=0.1 * np.ones(7),
        move_to_start_acceleration_limits=0.1 * np.ones(7),
    )
