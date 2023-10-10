import argparse
import logging

import numpy as np

from manipulation.station import load_scenario
from pydrake.all import (
    DiagramBuilder,
    MeshcatVisualizer,
    PiecewisePose,
    RigidTransform,
    RollPitchYaw,
    Simulator,
)

from iiwa_setup.controllers import OpenLoopPlanarPushingController
from iiwa_setup.iiwa import IiwaHardwareStationDiagram


def main(
    scenario_str: str,
    use_hardware: bool,
    html_path: str,
    pushing_start_pose: RigidTransform,
    pushing_pose_trajectory: PiecewisePose,
) -> None:
    builder = DiagramBuilder()

    scenario = load_scenario(data=scenario_str)
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
        ),
    )
    builder.Connect(
        station.GetOutputPort("iiwa.position_measured"),
        controller.GetInputPort("iiwa.position_measured"),
    )
    builder.Connect(
        station.GetOutputPort("iiwa.state_estimated"),
        controller.GetInputPort("iiwa.state_estimated"),
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
    simulator.set_target_realtime_rate(1.0)

    visualizer.StartRecording()
    station.internal_meshcat.AddButton("Stop Simulation")
    while (
        station.internal_meshcat.GetButtonClicks("Stop Simulation") < 1
        and not controller.is_finished()
    ):
        simulator.AdvanceTo(simulator.get_context().get_time() + 0.1)
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
        "--iiwa14", action="store_true", help="Whether to use iiwa14 instead of iiwa7."
    )
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

    scenario_str = f"""
    directives:
    - add_directives:
        file: package://iiwa_setup/iiwa{"14" if args.iiwa14 else "7"}.dmd.yaml
    plant_config:
        # For some reason, this requires a small timestep
        time_step: 0.0001
        contact_model: "hydroelastic"
        discrete_contact_solver: "sap"
    model_drivers:
        iiwa: !IiwaDriver {{}}
    """

    # TODO: The path should be passes as an argument
    poses = [
        RigidTransform(RollPitchYaw(np.pi, 0.0, -np.pi), [0.45, 0.0, 0.3]),
        RigidTransform(RollPitchYaw(np.pi, 0.0, -np.pi), [0.6, 0.0, 0.3]),
        RigidTransform(RollPitchYaw(np.pi, 0.0, -np.pi), [0.75, 0.0, 0.3]),
    ]
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
    )
