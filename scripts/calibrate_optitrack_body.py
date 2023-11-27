"""
Script for finding the transform between an optitrack body and a plant body. See the
README for usage instructions.
"""

import logging

import numpy as np

from manipulation.station import load_scenario
from pydrake.all import (
    AddFrameTriadIllustration,
    Diagram,
    DiagramBuilder,
    MultibodyPlant,
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
    Simulator,
)

from iiwa_setup.iiwa import IiwaHardwareStationDiagram
from iiwa_setup.sensors import OptitrackObjectTransformUpdaterDiagram
from iiwa_setup.util import BodyPoseLogger


def main(
    scenario_str: str,
    is_init: bool,
    object_name: str,
    ref_object_name: str,
    ref_object_initial_positions: np.ndarray,
    optitrack_iiwa_id: int,
    optitrack_body_id: int,
    X_optitrackBody_plantBody_world: RigidTransform,
):
    builder = DiagramBuilder()

    scenario = load_scenario(data=scenario_str)
    station: IiwaHardwareStationDiagram = builder.AddNamedSystem(
        "station",
        IiwaHardwareStationDiagram(
            scenario=scenario, has_wsg=False, use_hardware=False
        ),
    )

    # We need to use the simulated plant during initialization to determine the body's
    # z-position using static stability
    plant: MultibodyPlant = (
        station._external_station.GetSubsystemByName("plant")
        if is_init
        else station.get_internal_plant()
    )

    object_model_instance = plant.GetModelInstanceByName(object_name)
    object_body = plant.GetBodyByName(object_name + "_base_link", object_model_instance)
    AddFrameTriadIllustration(
        scene_graph=station.internal_scene_graph,
        body=object_body,
    )
    ref_object_model_instance = station.get_model_instance(ref_object_name)
    ref_object_body = plant.GetBodyByName(
        object_name + "_base_link",
        ref_object_model_instance,
    )
    AddFrameTriadIllustration(
        scene_graph=station.internal_scene_graph,
        body=ref_object_body,
    )

    # Add logger for logging the body's pose
    body_pose_logger: BodyPoseLogger = builder.AddNamedSystem(
        "body_pose_logger",
        BodyPoseLogger(
            body_frame=(ref_object_body if is_init else object_body).body_frame(),
            period_s=1.0,
        ),
    )

    if is_init:
        plant.SetDefaultPositions(
            ref_object_model_instance, ref_object_initial_positions
        )

        # Move object away from reference object to avoid collision
        object_initial_positions = ref_object_initial_positions
        object_initial_positions[-3] += 10.0
        plant.SetDefaultPositions(object_model_instance, object_initial_positions)
    else:
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

    diagram: Diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)

    body_pose_logger.set_plant_context(plant_context)
    if not is_init:
        optitrack_object_transform_updater.set_plant_context(plant_context)

    simulator = Simulator(diagram, context)

    simulator.set_target_realtime_rate(1.0)
    station.internal_meshcat.AddButton("Stop Simulation")
    while station.internal_meshcat.GetButtonClicks("Stop Simulation") < 1:
        simulator.AdvanceTo(simulator.get_context().get_time() + 0.1)


if __name__ == "__main__":
    logging.basicConfig(level="INFO")

    scenario_str = f"""
    directives:
    - add_directives:
        file: package://iiwa_setup/iiwa7_with_cylinder_pusher.dmd.yaml
    - add_directives:
        file: package://iiwa_setup/floor.dmd.yaml

    - add_directives:
        file: package://iiwa_setup/sugar_box.dmd.yaml
    - add_directives:
        file: package://iiwa_setup/sugar_box_reference.dmd.yaml

    plant_config:
        time_step: 0.001
        contact_model: "hydroelastic_with_fallback"
        discrete_contact_solver: "sap"
    model_drivers:
        iiwa: !IiwaDriver {{}}
    """

    is_init = True

    object_name = "sugar_box"
    ref_object_name = "sugar_box_reference"
    ref_object_initial_positions = np.array([1, 0, 0, 0, 0.3, 0, 0.025])
    optitrack_iiwa_id = 4
    optitrack_body_id = 3
    R_OptitrackBody_SimBody_W = RotationMatrix(RollPitchYaw([0.0, 0.0, 0.0]))
    p_OptitrackBody_SimBody_W = [0.0, 0.0, 0.0]

    main(
        scenario_str=scenario_str,
        is_init=is_init,
        object_name=object_name,
        ref_object_name=ref_object_name,
        ref_object_initial_positions=ref_object_initial_positions,
        optitrack_iiwa_id=optitrack_iiwa_id,
        optitrack_body_id=optitrack_body_id,
        X_optitrackBody_plantBody_world=RigidTransform(
            R_OptitrackBody_SimBody_W, p_OptitrackBody_SimBody_W
        ),
    )
