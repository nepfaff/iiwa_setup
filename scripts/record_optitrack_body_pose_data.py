import argparse
import json
import logging
import os

from typing import List

import numpy as np

from manipulation.station import LoadScenario
from pydrake.all import (
    AddFrameTriadIllustration,
    ApplySimulatorConfig,
    Diagram,
    DiagramBuilder,
    LogVectorOutput,
    MeshcatVisualizer,
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
    Simulator,
)

from iiwa_setup.iiwa import IiwaHardwareStationDiagram
from iiwa_setup.sensors import OptitrackObjectTransformUpdaterDiagram


def main(
    out_path: str,
    scenario_str: str,
    object_name: str,
    optitrack_body_id: int,
    optitrack_iiwa_id: int,
    object_initial_positions: List[float],
    X_optitrackBody_plantBody_world: RigidTransform,
    logging_frequency_hz: float,
    save_html: bool,
) -> None:
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
    plant.SetDefaultPositions(object_model_instance, object_initial_positions)

    object_body = plant.GetBodyByName(object_name + "_base_link", object_model_instance)
    AddFrameTriadIllustration(
        scene_graph=station.internal_scene_graph,
        body=object_body,
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
            ),
        )
    )

    object_pose_logger = LogVectorOutput(
        optitrack_object_transform_updater.GetOutputPort("object_positions"),
        builder,
        1.0 / logging_frequency_hz,
    )

    visualizer = MeshcatVisualizer.AddToBuilder(
        builder, station.GetOutputPort("query_object"), station.internal_meshcat
    )

    diagram: Diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)

    optitrack_object_transform_updater.set_plant_context(plant_context)

    simulator = Simulator(diagram, context)
    ApplySimulatorConfig(scenario.simulator_config, simulator)
    simulator.set_target_realtime_rate(1.0)

    station.internal_meshcat.AddButton("Stop Simulation")
    visualizer.StartRecording()
    while station.internal_meshcat.GetButtonClicks("Stop Simulation") < 1:
        simulator.AdvanceTo(context.get_time() + 0.1)
    visualizer.StopRecording()
    visualizer.PublishRecording()

    # Retrieve and save logs
    object_poses = object_pose_logger.FindLog(context).data().T  # Shape (t, 7)
    object_pose_times = object_pose_logger.FindLog(context).sample_times()  # Shape (t,)
    np.save(os.path.join(out_path, "object_poses.npy"), object_poses)
    np.savetxt(os.path.join(out_path, "object_pose_times.txt"), object_pose_times)

    if save_html is not None:
        html = station.internal_meshcat.StaticHtml()
        path = os.path.join(out_path, "meshcat.html")
        with open(path, "w") as f:
            f.write(html)


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
        "--object_initial_positions",
        type=json.loads,
        required=True,
        help="The initial object position with respect to the plant world of form "
        + "[qw, qx, qy, qz, x, y, z].",
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
        "--logging_frequency",
        type=float,
        default=1000,
        help="The object pose logging frequency in HZ.",
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

    scenario_str = f"""
    directives:
    - add_directives:
        file: package://iiwa_setup/iiwa7_with_cylinder_pusher.dmd.yaml
    - add_directives:
        file: {args.object_directive}
    plant_config:
        time_step: 0.001
        contact_model: "hydroelastic_with_fallback"
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

    main(
        out_path=out_path,
        scenario_str=scenario_str,
        object_name=args.object_name,
        optitrack_body_id=args.optitrack_object_id,
        optitrack_iiwa_id=args.optitrack_iiwa_id,
        object_initial_positions=args.object_initial_positions,
        X_optitrackBody_plantBody_world=RigidTransform(
            RotationMatrix(RollPitchYaw(args.R_optitrackBody_plantBody_world)),
            args.p_optitrackBody_plantBody_world,
        ),
        logging_frequency_hz=args.logging_frequency,
        save_html=args.save_html,
    )
