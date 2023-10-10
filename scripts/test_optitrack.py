import numpy as np

from manipulation.station import load_scenario
from optitrack import optitrack_frame_t, optitrack_rigid_body_t
from pydrake.all import (
    Diagram,
    DiagramBuilder,
    DrakeLcm,
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
    Simulator,
)

from iiwa_setup.iiwa import IiwaHardwareStationDiagram
from iiwa_setup.sensors import OptitrackObjectTransformUpdaterDiagram

# Calibration Data for sugar_box:
optitrack_iiwa_id = 4
optitrack_body_id = 3
R_OptitrackBody_SimBody = RotationMatrix(RollPitchYaw([0.02, 0, -np.pi / 2 - 0.2]))
p_OptitrackBody_SimBody = [0.6, 0.01, 0]
# Transform from optitrack body to simulation body
X_OB_SB = RigidTransform(R_OptitrackBody_SimBody, p_OptitrackBody_SimBody)
X_OB_SB = RigidTransform()
retain_z = True


def main(
    scenario_str: str,
    use_hardware: bool,
):
    builder = DiagramBuilder()

    scenario = load_scenario(data=scenario_str)
    station: IiwaHardwareStationDiagram = builder.AddNamedSystem(
        "station",
        IiwaHardwareStationDiagram(
            scenario=scenario, has_wsg=False, use_hardware=use_hardware
        ),
    )

    obpitrack_object_transform_updater: OptitrackObjectTransformUpdaterDiagram = (
        builder.AddNamedSystem(
            "OptitrackTransformUpdater",
            OptitrackObjectTransformUpdaterDiagram(
                station=station,
                object_name="sugar_box",
                optitrack_iiwa_id=optitrack_iiwa_id,
                optitrack_body_id=optitrack_body_id,
                retain_z=retain_z,
                X_optitrackBody_plantBody=X_OB_SB,
            ),
        )
    )

    diagram: Diagram = builder.Build()
    context = diagram.CreateDefaultContext()
    simulator = Simulator(diagram, context)
    simulator.set_target_realtime_rate(1.0)

    plant = station.get_plant()
    plant_context = plant.GetMyContextFromRoot(context)
    obpitrack_object_transform_updater.set_plant_context(plant_context)

    lcm = DrakeLcm()

    station.internal_meshcat.AddButton("Stop Simulation")
    while station.internal_meshcat.GetButtonClicks("Stop Simulation") < 1:
        # Publish optitrack msg
        iiwa_body_msg = optitrack_rigid_body_t()
        iiwa_body_msg.id = 4
        iiwa_body_msg.xyz = [0, 0, 0]
        iiwa_body_msg.quat = [0, 0, 0, 1]

        sugar_box_body_msg = optitrack_rigid_body_t()
        sugar_box_body_msg.id = 3
        sugar_box_body_msg.xyz = [0.5, 0, 0.3]
        sugar_box_body_msg.quat = [0, 0, 0, 1]

        msg = optitrack_frame_t()
        msg.num_rigid_bodies = 2
        msg.rigid_bodies = [iiwa_body_msg, sugar_box_body_msg]

        lcm.Publish(channel="OPTITRACK_FRAMES", buffer=msg.encode())

        simulator.AdvanceTo(simulator.get_context().get_time() + 0.1)

    station.internal_meshcat.DeleteButton("Stop Simulation")


if __name__ == "__main__":
    # TODO: Use argparser
    use_hardware = False

    # TODO: Need to add table to prevent object from falling
    scenario_str = f"""
    directives:
    - add_directives:
        file: package://iiwa_setup/iiwa14.dmd.yaml
    - add_directives:
        file: package://iiwa_setup/sugar_box.dmd.yaml
    - add_directives:
        file: package://iiwa_setup/floor.dmd.yaml
    plant_config:
        # For some reason, this requires a small timestep
        time_step: 0.0001
        contact_model: "hydroelastic"
        discrete_contact_solver: "sap"
    model_drivers:
        iiwa: !IiwaDriver {{}}
    """

    main(scenario_str=scenario_str, use_hardware=use_hardware)
