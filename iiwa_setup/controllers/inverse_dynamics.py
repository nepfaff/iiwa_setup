from typing import List, Optional

import numpy as np

from manipulation.station import MakeMultibodyPlant, Scenario
from pydrake.all import (
    Adder,
    Context,
    Diagram,
    DiagramBuilder,
    Gain,
    InverseDynamics,
    InverseDynamicsController,
    MultibodyPlant,
    SharedPointerSystem,
)

from iiwa_setup.util import get_package_xmls


class InverseDynamicsControllerWithGravityCompensationCancellation(Diagram):
    """
    An inverse dynamics controller that uses inverse dynamics to compute commanded
    torques from desired positions, velocities, and accelerations. It attempts to
    cancel out the iiwa control box's internal gravity compensation.
    NOTE: This assumes that the iiwa control box does not know about the gripper and
    thus only considers the arm in its gravity compensation.

    Input ports:
        - "iiwa.state_estimated": The estimated robot state [q, q_dot].
        - "iiwa.desired_state": The desired robot state [q, q_dot].
        - "iiwa.desired_accelerations": The desired robot accelerations q_ddot.

    Output ports:
        - "iiwa.desired_torque": The desired torques to send to the robot.
    """

    def __init__(
        self,
        scenario: Scenario,
        controller_plant: MultibodyPlant,
        controller_plant_context: Optional[Context] = None,
        kp_gains: np.ndarray = np.full(7, 600),
        damping_ratios: np.ndarray = np.full(7, 0.2),
        iiwa_model_instance_name: str = "iiwa",
        package_xmls: List[str] = [],
    ):
        """
        Args:
            scenario: The scenario in which the controller will be used.
            controller_plant: The controller plant for computing the inverse dynamics.
            controller_plant_context: A context for the controller plant with
                non-default dynamics parameters. More accurate parameters will improve
                controller performance and enable lower gains.
            kp_gains: The proportional gains for the controller.
            damping_ratios: The damping ratios for the controller.
            iiwa_model_instance_name: The model instance name of the iiwa in the
                scenario.
            package_xmls: The package XMLs needed to parse the scenario.
        """
        super().__init__()

        builder = DiagramBuilder()
        num_positions = controller_plant.num_positions()

        torque_adder: Adder = builder.AddNamedSystem(
            "torque_adder", Adder(2, num_positions)
        )
        builder.ExportOutput(torque_adder.get_output_port(), "iiwa.desired_torque")

        # Inverse dunamics control
        inverse_dynamics_controller: InverseDynamicsController = builder.AddNamedSystem(
            "inverse_dynamics_controller",
            InverseDynamicsController(
                controller_plant,
                kp=kp_gains,
                ki=[1] * num_positions,
                kd=2 * damping_ratios * np.sqrt(kp_gains),
                has_reference_acceleration=True,
                plant_context=controller_plant_context,
            ),
        )
        builder.ExportInput(
            inverse_dynamics_controller.get_input_port_desired_state(),
            "iiwa.desired_state",
        )
        builder.ExportInput(
            inverse_dynamics_controller.get_input_port_desired_acceleration(),
            "iiwa.desired_accelerations",
        )
        builder.ExportInput(
            inverse_dynamics_controller.get_input_port_estimated_state(),
            "iiwa.state_estimated",
        )
        builder.Connect(
            inverse_dynamics_controller.get_output_port_control(),
            torque_adder.get_input_port(0),
        )

        # Cancel out the iiwa control box's internal gravity compensation.
        iiwa_only_controller_plant = MakeMultibodyPlant(
            scenario=scenario,
            model_instance_names=[iiwa_model_instance_name],
            package_xmls=get_package_xmls() + package_xmls,
        )
        # Keep the controller plant alive during the Diagram lifespan.
        builder.AddNamedSystem(
            "iiwa_only_controller_plant_pointer_system",
            SharedPointerSystem(iiwa_only_controller_plant),
        )
        gravity_compensation: InverseDynamics = builder.AddNamedSystem(
            "gravity_compensation",
            InverseDynamics(
                plant=iiwa_only_controller_plant,
                mode=InverseDynamics.InverseDynamicsMode.kGravityCompensation,
            ),
        )
        builder.ConnectToSame(
            inverse_dynamics_controller.get_input_port_estimated_state(),
            gravity_compensation.get_input_port_estimated_state(),
        )
        negater: Gain = builder.AddNamedSystem(
            "negater", Gain(k=-1, size=num_positions)
        )
        builder.Connect(
            gravity_compensation.get_output_port(),
            negater.get_input_port(),
        )
        builder.Connect(
            negater.get_output_port(),
            torque_adder.get_input_port(1),
        )

        builder.BuildInto(self)
