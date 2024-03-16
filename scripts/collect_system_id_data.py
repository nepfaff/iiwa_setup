"""Script for collecting (q, qdot, tau) trajectory data for system identification."""

import argparse
import json
import logging

from pathlib import Path
from typing import List, Tuple

import numpy as np

from manipulation.station import load_scenario
from pydrake.all import (
    CompositeTrajectory,
    DiagramBuilder,
    LogVectorOutput,
    MeshcatVisualizer,
    PiecewisePolynomial,
    PiecewisePose,
    RigidTransform,
    RollPitchYaw,
    Simulator,
    Trajectory,
)

from iiwa_setup.controllers import SystemIDDataCollectionController
from iiwa_setup.iiwa import IiwaHardwareStationDiagram
from iiwa_setup.motion_planning import reparameterize_with_toppra
from iiwa_setup.util import NoDrakeDifferentialIKFilter


# TODO: Try "Data Collection" strategy from "End-to-End Learning of Hybrid Inverse
# Dynamics Models for Precise and Compliant Impedance Control." paper
def generate_sinusoidal_traj_andy(
    initial_positions: List[float],
    base_frequency_idx: int,
    amplitude: float,
    duration_s: float,
    timestep_s: float,
) -> Tuple[PiecewisePolynomial, np.ndarray, np.ndarray]:
    """Generates a sinusoidal joint space trajectory for a robotic manipulator, where
    every joint moves at a different frequency. The sinusoidal form is taken from
    Andy Lambert's work.

    Args:
        initial_positions (List[float]): The initial joint positions. The length of the
            lists corresponds to the number of joints.
        base_frequency_idx (int): The base frequency index to use.
        amplitude (float): The sinusoidal amplitude.
        duration_s (float): The trajectory duration in seconds.
        timestep_s (float): The time step/ sample period for converting the continuous
            trajectory into a piecewise polynomial.

    Returns:
        Tuple[PiecewisePolynomial, np.ndarray, np.ndarray]: A tuple of the the generated
            trajectory, the joint positions, and the sample times.
    """
    num_joints = len(initial_positions)
    sample_times_s = np.arange(0, duration_s, timestep_s)
    joint_positions = []
    for i in range(num_joints):
        joint_positions.append(
            initial_positions[i]
            + amplitude * np.sin((base_frequency_idx + i) * 0.5 * sample_times_s)
        )
    joint_positions = np.array(joint_positions)
    return (
        PiecewisePolynomial.CubicShapePreserving(
            sample_times_s, joint_positions, zero_end_point_derivatives=True
        ),
        joint_positions,
        sample_times_s,
    )


def main(
    scenario_str: str,
    use_hardware: bool,
    out_dir: Path,
    html_path: str,
    system_id_joint_traj: Trajectory,
    move_to_start_velocity_limits: np.ndarray,
    move_to_start_acceleration_limits: np.ndarray,
    system_id_velocity_limits: np.ndarray,
    system_id_acceleration_limits: np.ndarray,
    num_joint_positions=7,
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

    # Reparameterize the trajectory with Toppra to ensure that it is executable
    retimed_system_id_joint_traj = reparameterize_with_toppra(
        trajectory=system_id_joint_traj,
        plant=controller_plant,
        velocity_limits=system_id_velocity_limits,
        acceleration_limits=system_id_acceleration_limits,
    )

    controller: SystemIDDataCollectionController = builder.AddNamedSystem(
        "system_id_data_collection_controller",
        SystemIDDataCollectionController(
            controller_plant=controller_plant,
            num_joint_positions=num_joint_positions,
            system_id_joint_traj=retimed_system_id_joint_traj,
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

    state_logger = LogVectorOutput(
        station.GetOutputPort("iiwa.state_estimated"), builder
    )
    torque_logger = LogVectorOutput(
        station.GetOutputPort("iiwa.torque_commanded"), builder
    )

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    context = simulator.get_context()

    visualizer.StartRecording()
    while not controller.is_finished():
        simulator.AdvanceTo(context.get_time() + 0.1)
    visualizer.StopRecording()
    visualizer.PublishRecording()

    print(f"Actual simulation realtime rate: {simulator.get_actual_realtime_rate()}")

    if html_path is not None:
        html = station.internal_meshcat.StaticHtml()
        with open(html_path, "w") as f:
            f.write(html)

    # Retrieve logs
    state_logs = state_logger.FindLog(context)
    torque_logs = torque_logger.FindLog(context)
    sample_times = state_logs.sample_times()
    state_logs_data = state_logs.data().T
    torque_logs_data = torque_logs.data().T

    # Only keep system ID trajectory logs
    system_id_joint_traj_start_time = controller.get_system_id_joint_traj_start_time()
    starting_idx = np.argmax(sample_times >= system_id_joint_traj_start_time)
    system_id_state_logs = state_logs_data[starting_idx:]
    system_id_torque_logs = torque_logs_data[starting_idx:]

    np.save(out_dir / "system_id_state_logs.npy", system_id_state_logs)
    np.save(out_dir / "system_id_torque_logs.npy", system_id_torque_logs)
    np.save(
        out_dir / "system_id_log_sample_times.npy",
        sample_times[: len(system_id_state_logs)],
    )


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out_dir",
        type=str,
        required=True,
        help="The directory to save the system ID data to.",
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
        "--system_id_velocity_limits",
        type=json.loads,
        default="[0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3]",
        help="The velocity limits for the system ID trajectory.",
    )
    parser.add_argument(
        "--system_id_acceleration_limits",
        type=json.loads,
        default="[0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3]",
        help="The acceleration limits for the system ID trajectory.",
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

    time_step = 0.001
    scenario_str = f"""
    directives:
    - add_directives:
        file: package://iiwa_setup/iiwa7_with_planar_pusher.dmd.yaml
    plant_config:
        # Cannot use bigger timesteps on the real robot
        time_step: {time_step}
        contact_model: "hydroelastic"
        discrete_contact_solver: "sap"
    model_drivers:
        iiwa: !IiwaDriver {{}}
    """

    system_id_joint_traj = PiecewisePolynomial.CubicShapePreserving(
        np.array([0.0, 1.0]),
        np.array(
            [
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.4],
            ]
        ).T,
        zero_end_point_derivatives=True,
    )
    # TODO: Validate that no collisions before executing on the real robot
    # TODO: This is a lot slower than real time at the moment. Is this a problem when
    # switching to hardware? Can we pre-compute this?
    initial_positions = [0.0, 0.1, 0.0, -1.2, 0.0, 1.0, 0.0]
    system_id_joint_trajs: List[PiecewisePolynomial] = []
    combined_joint_positions, combined_sample_times_s = [], []
    num_trajs = 10
    for i in range(num_trajs):
        last_traj = system_id_joint_trajs[-1] if i > 0 else None
        traj, joint_positions, sample_times_s = generate_sinusoidal_traj_andy(
            initial_positions=last_traj.value(last_traj.end_time())
            if last_traj is not None
            else initial_positions,
            amplitude=1.0,
            duration_s=10.0,
            base_frequency_idx=i,
            timestep_s=time_step * 100,  # Speed up simulation
        )
        combined_joint_positions.append(joint_positions)
        shift_time = last_traj.end_time() if i > 0 else 0.0
        combined_sample_times_s.append(sample_times_s + shift_time)
        traj.shiftRight(shift_time)
        system_id_joint_trajs.append(traj)
    combined_joint_positions = np.concatenate(combined_joint_positions, axis=1)
    combined_sample_times_s = np.concatenate(combined_sample_times_s)
    system_id_joint_traj = CompositeTrajectory(system_id_joint_trajs)

    out_dir = Path(args.out_dir)
    out_dir.mkdir(exist_ok=True)
    np.save(out_dir / "desired_joint_positions.npy", combined_joint_positions)
    np.save(out_dir / "desired_joint_positions_times.npy", combined_sample_times_s)

    main(
        scenario_str=scenario_str,
        use_hardware=args.use_hardware,
        out_dir=out_dir,
        html_path=args.html_path,
        system_id_joint_traj=system_id_joint_traj,
        move_to_start_velocity_limits=0.3 * np.ones(7),
        move_to_start_acceleration_limits=0.3 * np.ones(7),
        system_id_velocity_limits=np.array(args.system_id_velocity_limits),
        system_id_acceleration_limits=np.array(args.system_id_acceleration_limits),
    )
