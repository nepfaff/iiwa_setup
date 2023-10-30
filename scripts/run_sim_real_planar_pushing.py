"""
Script for comparing simulated and real planar pushing rollouts.
The simulation is executed first and the exact same iiwa joint positions are then
executed on the real hardware. Object poses are logged for both sim and real experiments.
"""

import argparse
import logging
import os

from typing import Tuple

import numpy as np

from manipulation.station import load_scenario
from pydrake.all import (
    DiagramBuilder,
    LogVectorOutput,
    MeshcatVisualizer,
    PiecewisePolynomial,
    PiecewisePose,
    RigidTransform,
    RollPitchYaw,
    Simulator,
    TrajectorySource,
)

from iiwa_setup.controllers import (
    OpenLoopPlanarPushingController,
    PlanAndMoveToPositionsUnconstrainedController,
)
from iiwa_setup.iiwa import IiwaHardwareStationDiagram
from iiwa_setup.util import NoDrakeDifferentialIKFilter


def move_real(
    scenario_str: str,
    use_hardware: bool,
    logging_path: str,
    save_html: bool,
    pushing_start_pose: RigidTransform,
    pushing_pose_trajectory: PiecewisePose,
    move_to_start_velocity_limits: np.ndarray,
    move_to_start_acceleration_limits: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute and execute the open loop planar pushing trajectory on the real iiwa.
    Returns the saved joint positions that were commanded to enable replaying the exact
    trajectory.
    """
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

    iiwa_position_logger = LogVectorOutput(
        station.GetOutputPort("iiwa.position_commanded"),
        builder,
        scenario.plant_config.time_step,
    )

    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, station.GetOutputPort("query_object"), station.internal_meshcat
    )

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)

    context = simulator.get_context()
    controller.set_context(context)

    visualizer.StartRecording()
    while not controller.is_finished():
        simulator.AdvanceTo(context.get_time() + 0.1)
    visualizer.StopRecording()
    visualizer.PublishRecording()

    iiwa_positions = iiwa_position_logger.FindLog(context).data().T  # Shape (t, 7)
    iiwa_position_times = iiwa_position_logger.FindLog(
        context
    ).sample_times()  # Shape (t,)
    np.save(os.path.join(logging_path, "real_iiwa_positions.npy"), iiwa_positions)
    np.savetxt(
        os.path.join(logging_path, "real_iiwa_position_times.txt"), iiwa_position_times
    )

    if save_html:
        html = station.internal_meshcat.StaticHtml()
        with open(os.path.join(logging_path, "real_meshcat.html"), "w") as f:
            f.write(html)

    return iiwa_positions, iiwa_position_times


def move_sim_to_start(
    scenario_str: str,
    start_positions: np.ndarray,
    velocity_limits: np.ndarray,
    acceleration_limits: np.ndarray,
) -> None:
    """Move the simulated iiwa to the initial positions of the real iiwa."""
    builder = DiagramBuilder()

    scenario = load_scenario(data=scenario_str)
    station: IiwaHardwareStationDiagram = builder.AddNamedSystem(
        "station",
        IiwaHardwareStationDiagram(
            scenario=scenario, has_wsg=False, use_hardware=False
        ),
    )

    controller_plant = station.get_iiwa_controller_plant()
    plan_and_move_to_positions_controller: PlanAndMoveToPositionsUnconstrainedController = builder.AddNamedSystem(
        "plan_and_move_to_positions_controller",
        PlanAndMoveToPositionsUnconstrainedController(
            controller_plant=controller_plant,
            goal_positions=start_positions,
            velocity_limits=velocity_limits,
            acceleration_limits=acceleration_limits,
        ),
    )
    builder.Connect(
        station.GetOutputPort("iiwa.position_measured"),
        plan_and_move_to_positions_controller.get_input_port(),
    )
    builder.Connect(
        plan_and_move_to_positions_controller.get_output_port(),
        station.GetInputPort("iiwa.position"),
    )

    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, station.GetOutputPort("query_object"), station.internal_meshcat
    )

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    context = simulator.get_context()

    visualizer.StartRecording()
    while not plan_and_move_to_positions_controller.is_finished():
        simulator.AdvanceTo(context.get_time() + 0.1)
    visualizer.StopRecording()
    visualizer.PublishRecording()


def move_sim(
    scenario_str: str,
    logging_path: str,
    save_html: bool,
    iiwa_positions: np.ndarray,
    iiwa_position_times: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Move the simulated iiwa according to the recorded joint positions."""
    builder = DiagramBuilder()

    scenario = load_scenario(data=scenario_str)
    station: IiwaHardwareStationDiagram = builder.AddNamedSystem(
        "station",
        IiwaHardwareStationDiagram(
            scenario=scenario, has_wsg=False, use_hardware=False
        ),
    )

    iiwa_position_source: TrajectorySource = builder.AddNamedSystem(
        "iiwa_position_source",
        TrajectorySource(
            PiecewisePolynomial.ZeroOrderHold(
                breaks=iiwa_position_times, samples=iiwa_positions.T
            )
        ),
    )
    builder.Connect(
        iiwa_position_source.get_output_port(),
        station.GetInputPort("iiwa.position"),
    )

    iiwa_position_logger = LogVectorOutput(
        station.GetOutputPort("iiwa.position_commanded"),
        builder,
        scenario.plant_config.time_step,
    )

    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, station.GetOutputPort("query_object"), station.internal_meshcat
    )

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)

    context = simulator.get_context()

    visualizer.StartRecording()
    simulator.AdvanceTo(iiwa_position_times[-1])
    visualizer.StopRecording()
    visualizer.PublishRecording()

    iiwa_positions = iiwa_position_logger.FindLog(context).data().T  # Shape (t, 7)
    iiwa_position_times = iiwa_position_logger.FindLog(
        context
    ).sample_times()  # Shape (t,)
    np.save(os.path.join(logging_path, "sim_iiwa_positions.npy"), iiwa_positions)
    np.savetxt(
        os.path.join(logging_path, "sim_iiwa_position_times.txt"),
        iiwa_position_times,
    )

    if save_html:
        html = station.internal_meshcat.StaticHtml()
        with open(os.path.join(logging_path, "sim_meshcat.html"), "w") as f:
            f.write(html)

    return iiwa_positions, iiwa_position_times


def main(
    scenario_str: str,
    use_hardware: bool,
    logging_path: str,
    save_html: bool,
    pushing_start_pose: RigidTransform,
    pushing_pose_trajectory: PiecewisePose,
    move_to_start_velocity_limits: np.ndarray,
    move_to_start_acceleration_limits: np.ndarray,
) -> None:
    real_iiwa_positions, real_iiwa_position_times = move_real(
        scenario_str=scenario_str,
        use_hardware=use_hardware,
        logging_path=logging_path,
        save_html=save_html,
        pushing_start_pose=pushing_start_pose,
        pushing_pose_trajectory=pushing_pose_trajectory,
        move_to_start_velocity_limits=move_to_start_velocity_limits,
        move_to_start_acceleration_limits=move_to_start_acceleration_limits,
    )

    move_sim_to_start(
        scenario_str=scenario_str,
        start_positions=real_iiwa_positions[0],
        velocity_limits=move_to_start_velocity_limits,
        acceleration_limits=move_to_start_acceleration_limits,
    )

    sim_iiwa_positions, _ = move_sim(
        scenario_str=scenario_str,
        logging_path=logging_path,
        save_html=save_html,
        iiwa_positions=real_iiwa_positions,
        iiwa_position_times=real_iiwa_position_times,
    )

    if np.allclose(real_iiwa_positions, sim_iiwa_positions):
        logging.info("Real and simulated iiwa positions are close.")
    else:
        logging.warning("Real and simulated iiwa positions are not close!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--use_hardware",
        action="store_true",
        help="Whether to use real world hardware.",
    )
    parser.add_argument(
        "--out_path",
        type=str,
        required=True,
        help="The path to the directory to write the results to.",
    )
    parser.add_argument(
        "--save_html",
        action="store_true",
        help="Whether to save the meschat recording HTML.",
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
        file: package://iiwa_setup/iiwa7_with_cylinder_pusher.dmd.yaml
    plant_config:
        # Cannot use bigger timesteps on the real robot
        time_step: 0.001
        contact_model: "hydroelastic"
        discrete_contact_solver: "sap"
    model_drivers:
        iiwa: !IiwaDriver {{}}
    """

    out_path = args.out_path
    if not os.path.exists(out_path):
        os.mkdir(out_path)

    # TODO: The path should be passed as an argument
    poses = [
        RigidTransform(RollPitchYaw(np.pi, 0.0, -np.pi), [0.5, 0.0, 0.25]),
        RigidTransform(RollPitchYaw(np.pi, 0.0, -np.pi), [0.6, 0.0, 0.25]),
        RigidTransform(RollPitchYaw(np.pi, 0.0, -np.pi), [0.7, 0.0, 0.25]),
    ]
    pushing_pose_traj = PiecewisePose.MakeLinear(
        times=np.arange(len(poses)),
        poses=poses,
    )

    main(
        scenario_str=scenario_str,
        use_hardware=args.use_hardware,
        logging_path=out_path,
        save_html=args.save_html,
        pushing_start_pose=poses[0],
        pushing_pose_trajectory=pushing_pose_traj,
        move_to_start_velocity_limits=0.1 * np.ones(7),
        move_to_start_acceleration_limits=0.1 * np.ones(7),
    )
