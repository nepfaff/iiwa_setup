"""
Script for comparing simulated and real planar pushing rollouts.
The real hardware is executed first and the exact same iiwa joint positions are then
executed in simulation. Object poses are logged for both sim and real experiments.
The optitrack object poses are simulated naively if `use_hardware` is False.
"""

import argparse
import json
import logging
import os

from typing import List, Tuple

import numpy as np

from manipulation.station import LoadScenario
from optitrack import optitrack_frame_t, optitrack_rigid_body_t
from pydrake.all import (
    ApplySimulatorConfig,
    DiagramBuilder,
    LogVectorOutput,
    MeshcatVisualizer,
    PiecewisePolynomial,
    PiecewisePose,
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
    Simulator,
    TrajectorySource,
)

from iiwa_setup.controllers import (
    OpenLoopPlanarPushingController,
    PlanAndMoveToPositionsUnconstrainedController,
)
from iiwa_setup.iiwa import IiwaHardwareStationDiagram
from iiwa_setup.sensors import OptitrackObjectTransformUpdaterDiagram
from iiwa_setup.util import NoDrakeDifferentialIKFilter


def move_real(
    scenario_str: str,
    use_hardware: bool,
    logging_path: str,
    object_name: str,
    optitrack_iiwa_id: int,
    optitrack_body_id: int,
    X_optitrackBody_plantBody_world: RigidTransform,
    save_html: bool,
    pushing_start_pose: RigidTransform,
    pushing_pose_trajectory: PiecewisePose,
    move_to_start_velocity_limits: np.ndarray,
    move_to_start_acceleration_limits: np.ndarray,
    lcm_publish_period: float,
    optitrack_frames: List[optitrack_frame_t],
    optitrack_frame_times: List[float],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Compute and execute the open loop planar pushing trajectory on the real iiwa.
    Returns the saved joint positions and joint position times that were commanded to
    enable replaying the exact trajectory. Also returns the initial object positions as
    determined by optitrack.
    """
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

    optitrack_object_transform_updater: OptitrackObjectTransformUpdaterDiagram = (
        builder.AddNamedSystem(
            "OptitrackTransformUpdater",
            OptitrackObjectTransformUpdaterDiagram(
                station=station,
                object_name=object_name,
                optitrack_iiwa_id=optitrack_iiwa_id,
                optitrack_body_id=optitrack_body_id,
                X_optitrackBody_plantBody_world=X_optitrackBody_plantBody_world,
                simulate=not use_hardware,
                lcm_publish_period=lcm_publish_period,
                optitrack_frames=optitrack_frames,
                optitrack_frame_times=optitrack_frame_times,
            ),
        )
    )

    iiwa_position_logger = LogVectorOutput(
        station.GetOutputPort("iiwa.position_commanded"),
        builder,
        scenario.plant_config.time_step,
    )
    object_state_logger = LogVectorOutput(
        station.GetOutputPort(f"{object_name}_state"),
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

    plant_context = station.get_internal_plant_context()
    optitrack_object_transform_updater.set_plant_context(plant_context)

    visualizer.StartRecording()
    while not controller.is_finished():
        simulator.AdvanceTo(context.get_time() + 0.1)
    visualizer.StopRecording()
    visualizer.PublishRecording()

    iiwa_positions = iiwa_position_logger.FindLog(context).data().T  # Shape (t, 7)
    iiwa_position_times = iiwa_position_logger.FindLog(
        context
    ).sample_times()  # Shape (t,)
    object_states = (
        object_state_logger.FindLog(context).data().T.copy()
    )  # Shape (t, 14)
    if not use_hardware:
        # The first object state is not valid when simulating optitrack
        object_states[0] = object_states[1]
    np.save(os.path.join(logging_path, "real_iiwa_positions.npy"), iiwa_positions)
    np.savetxt(
        os.path.join(logging_path, "real_iiwa_position_times.txt"), iiwa_position_times
    )
    np.save(os.path.join(logging_path, "real_object_states.npy"), object_states)

    if save_html:
        html = station.internal_meshcat.StaticHtml()
        with open(os.path.join(logging_path, "real_meshcat.html"), "w") as f:
            f.write(html)

    initial_object_positions = object_states[0, :7]
    return iiwa_positions, iiwa_position_times, initial_object_positions


def move_sim_to_start(
    scenario_str: str,
    start_positions: np.ndarray,
    velocity_limits: np.ndarray,
    acceleration_limits: np.ndarray,
) -> None:
    """Move the simulated iiwa to the initial positions of the real iiwa."""
    builder = DiagramBuilder()

    scenario = LoadScenario(data=scenario_str)
    station: IiwaHardwareStationDiagram = builder.AddNamedSystem(
        "station",
        IiwaHardwareStationDiagram(
            scenario=scenario, has_wsg=False, use_hardware=False
        ),
    )

    controller_plant = station.get_iiwa_controller_plant()
    plan_and_move_to_positions_controller: (
        PlanAndMoveToPositionsUnconstrainedController
    ) = builder.AddNamedSystem(
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
    object_name: str,
    initial_object_positions: np.ndarray,
) -> Tuple[np.ndarray, np.ndarray]:
    """Move the simulated iiwa according to the recorded joint positions."""
    builder = DiagramBuilder()

    scenario = LoadScenario(data=scenario_str)
    station: IiwaHardwareStationDiagram = builder.AddNamedSystem(
        "station",
        IiwaHardwareStationDiagram(
            scenario=scenario, has_wsg=False, use_hardware=False
        ),
    )

    plant = station.get_internal_plant()
    object_model_instance = plant.GetModelInstanceByName(object_name)
    plant.SetDefaultPositions(object_model_instance, initial_object_positions)

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
    object_state_logger = LogVectorOutput(
        station.GetOutputPort(f"{object_name}_state"),
        builder,
        scenario.plant_config.time_step,
    )

    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, station.GetOutputPort("query_object"), station.internal_meshcat
    )

    diagram = builder.Build()
    simulator = Simulator(diagram)
    ApplySimulatorConfig(scenario.simulator_config, simulator)
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
    object_states = object_state_logger.FindLog(context).data().T  # Shape (t, 14)
    np.save(os.path.join(logging_path, "sim_iiwa_positions.npy"), iiwa_positions)
    np.savetxt(
        os.path.join(logging_path, "sim_iiwa_position_times.txt"),
        iiwa_position_times,
    )
    np.save(os.path.join(logging_path, "sim_object_states.npy"), object_states)

    if save_html:
        html = station.internal_meshcat.StaticHtml()
        with open(os.path.join(logging_path, "sim_meshcat.html"), "w") as f:
            f.write(html)

    return iiwa_positions, iiwa_position_times


def main(
    scenario_str: str,
    use_hardware: bool,
    logging_path: str,
    object_name: str,
    optitrack_iiwa_id: int,
    optitrack_body_id: int,
    X_optitrackBody_plantBody_world: RigidTransform,
    save_html: bool,
    pushing_start_pose: RigidTransform,
    pushing_pose_trajectory: PiecewisePose,
    move_to_start_velocity_limits: np.ndarray,
    move_to_start_acceleration_limits: np.ndarray,
    lcm_publish_period: float,
    optitrack_frames: List[optitrack_frame_t],
    optitrack_frame_times: List[float],
) -> None:
    real_iiwa_positions, real_iiwa_position_times, initial_object_positions = move_real(
        scenario_str=scenario_str,
        use_hardware=use_hardware,
        logging_path=logging_path,
        object_name=object_name,
        optitrack_iiwa_id=optitrack_iiwa_id,
        optitrack_body_id=optitrack_body_id,
        X_optitrackBody_plantBody_world=X_optitrackBody_plantBody_world,
        save_html=save_html,
        pushing_start_pose=pushing_start_pose,
        pushing_pose_trajectory=pushing_pose_trajectory,
        move_to_start_velocity_limits=move_to_start_velocity_limits,
        move_to_start_acceleration_limits=move_to_start_acceleration_limits,
        lcm_publish_period=lcm_publish_period,
        optitrack_frames=optitrack_frames,
        optitrack_frame_times=optitrack_frame_times,
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
        object_name=object_name,
        initial_object_positions=initial_object_positions,
    )

    if np.allclose(real_iiwa_positions, sim_iiwa_positions):
        logging.info("Real and simulated iiwa positions are close.")
    else:
        logging.warning("Real and simulated iiwa positions are not close!")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out_path",
        type=str,
        required=True,
        help="The path to the directory to write the results to.",
    )
    parser.add_argument(
        "--object_directive",
        type=str,
        required=True,
        help="The package path of the direcetive that contains the object of interest.",
    )
    parser.add_argument(
        "--object_name",
        type=str,
        required=True,
        help="The name of the object. This must correspond to the name in "
        + "`object_directive`.",
    )
    parser.add_argument(
        "--optitrack_object_id",
        type=int,
        required=True,
        help="The optitrack object ID.",
    )
    parser.add_argument(
        "--optitrack_iiwa_id", type=int, default=4, help="The optitrack iiwa ID."
    )
    parser.add_argument(
        "--p_optitrackBody_plantBody_world",
        type=json.loads,
        required=True,
        help="The position component of X_optitrackBody_plantBody_world in form "
        + "[x, y, z].",
    )
    parser.add_argument(
        "--R_optitrackBody_plantBody_world",
        type=json.loads,
        required=True,
        help="The rotation component of X_optitrackBody_plantBody_world in form "
        + "[qw, qx, qy, qz].",
    )
    parser.add_argument(
        "--use_hardware",
        action="store_true",
        help="Whether to use real world hardware.",
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

    # Cannot use bigger timesteps on the real robot
    timestep = 0.001
    scenario_str = f"""
    directives:
    - add_directives:
        file: package://iiwa_setup/iiwa7_with_planar_pusher.dmd.yaml
    - add_directives:
        file: package://iiwa_setup/floor.dmd.yaml
    - add_directives:
        file: {args.object_directive}
    plant_config:
        time_step: {timestep}
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

    out_path = args.out_path
    if not os.path.exists(out_path):
        os.mkdir(out_path)

    poses = [
        RigidTransform(RollPitchYaw(np.pi, 0.0, -np.pi), [0.4, 0.0, 0.175]),
        RigidTransform(RollPitchYaw(np.pi, 0.0, -np.pi), [0.5, 0.0, 0.175]),
        RigidTransform(RollPitchYaw(np.pi, 0.0, -np.pi), [0.6, 0.0, 0.175]),
        RigidTransform(RollPitchYaw(np.pi, 0.0, -np.pi), [0.7, 0.0, 0.175]),
        RigidTransform(RollPitchYaw(np.pi, 0.0, -np.pi), [0.75, 0.0, 0.175]),
    ]

    # Rotate the gripper by 90 degrees around yaw in the gripper frame
    X_GG1 = RigidTransform(RollPitchYaw(0.0, 0.0, np.pi / 2.0), np.zeros(3))
    poses = [pose @ X_GG1 for pose in poses]

    pushing_pose_traj = PiecewisePose.MakeLinear(
        times=np.arange(len(poses)),
        poses=poses,
    )

    # Simulated optitrack frames for use if 'use_hardware' is False
    object_body = optitrack_rigid_body_t()
    object_body.id = args.optitrack_object_id
    object_body.xyz = [0.5, 0.0, 0.15]
    object_body.quat = [0.0, 0.0, 0.0, 1.0]
    iiwa_body = optitrack_rigid_body_t()
    iiwa_body.id = args.optitrack_iiwa_id
    iiwa_body.xyz = [0.0, 0.0, 0.0]
    iiwa_body.quat = [0.0, 0.0, 0.0, 1.0]
    optitrack_rigid_bodies = [object_body, iiwa_body]
    optitrack_frame = optitrack_frame_t()
    optitrack_frame.num_rigid_bodies = len(optitrack_rigid_bodies)
    optitrack_frame.rigid_bodies = optitrack_rigid_bodies
    optitrack_frames = [optitrack_frame]
    optitrack_frame_times = [0.0]

    main(
        scenario_str=scenario_str,
        use_hardware=args.use_hardware,
        logging_path=out_path,
        object_name=args.object_name,
        optitrack_body_id=args.optitrack_object_id,
        optitrack_iiwa_id=args.optitrack_iiwa_id,
        X_optitrackBody_plantBody_world=RigidTransform(
            RotationMatrix(RollPitchYaw(args.R_optitrackBody_plantBody_world)),
            args.p_optitrackBody_plantBody_world,
        ),
        save_html=args.save_html,
        pushing_start_pose=poses[0],
        pushing_pose_trajectory=pushing_pose_traj,
        move_to_start_velocity_limits=0.1 * np.ones(7),
        move_to_start_acceleration_limits=0.1 * np.ones(7),
        lcm_publish_period=timestep,
        optitrack_frames=optitrack_frames,
        optitrack_frame_times=optitrack_frame_times,
    )
