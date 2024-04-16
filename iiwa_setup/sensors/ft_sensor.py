import logging

try:
    from ft_reading_t import ft_reading_t
except ImportError:
    logging.warning("ft_reading_t module not found. Skipping import.")

import numpy as np

from pydrake.all import (
    AbstractValue,
    BasicVector,
    Context,
    LeafSystem,
    MultibodyPlant,
    SpatialForce,
)


class FTSensorDataReceiver(LeafSystem):
    """System for receiving F/T sensor data.

    Input ports:
        - ft_reading: The F/T sensor reading. This should be connected to the output of
            the LcmSubscriberSystem that subscribes to the F/T sensor data.
    Output ports:
        - time_measured: The timestamp at which the F/T sensor reading was measured.
        - ft_measured: The F/T sensor reading.
    """

    def __init__(self):
        super().__init__()

        self.DeclareAbstractInputPort("ft_reading", AbstractValue.Make(ft_reading_t()))

        self.DeclareVectorOutputPort(
            "time_measured", size=1, calc=self._calc_time_measured
        )
        self.DeclareVectorOutputPort("ft_measured", size=6, calc=self._calc_ft_measured)

    def _calc_time_measured(self, context: Context, output: BasicVector) -> None:
        ft_reading = self.get_input_port().Eval(context)
        output.set_value([ft_reading.timestamp])

    def _calc_ft_measured(self, context: Context, output: BasicVector) -> None:
        ft_reading = self.get_input_port().Eval(context)
        output.set_value(
            [
                ft_reading.fx,
                ft_reading.fy,
                ft_reading.fz,
                ft_reading.tx,
                ft_reading.ty,
                ft_reading.tz,
            ]
        )


class ForceSensorEvaluator(LeafSystem):
    """
    A system whose input port takes in MBP joint reaction forces and whose outputs
    represent the simulated F/T sensor measurements.
    See https://drake.mit.edu/doxygen_cxx/group__sensor__systems.html for more details
    about simulating F/T sensors.

    Input port:
        spatial_forces_in: The spatial forces acting on the joints of the MBP. This
            should be connected to `plant.get_reaction_forces_output_port()`.

    Output port:
        ft_measured: The F/T sensor measurements of form [f_x, f_y, f_z, tau_x, tau_y,
            tau_z].
    """

    def __init__(self, plant: MultibodyPlant, weldjoint_name: str):
        """
        Args:
            plant (MultibodyPlant): The plant containing the joint corresponding to
                `weldjoint_name`.
            weldjoint_name (str): The name of the weld joint that is used to model the
                F/T sensor.
        """
        super().__init__()

        ft_sensor_weldjoint = plant.GetJointByName(weldjoint_name)
        self._ft_sensor_weldjoint_idx = ft_sensor_weldjoint.index()

        self.DeclareAbstractInputPort(
            "spatial_forces_in", AbstractValue.Make([SpatialForce()])
        )

        self.DeclareVectorOutputPort("ft_measured", size=6, calc=self._calc_ft_measured)

    def _calc_ft_measured(self, context: Context, output: BasicVector) -> None:
        # Compute the reaction force F_CJc_Jc on the child body C at the joint's child
        # frame Jc.
        F_CJc_Jc: SpatialForce = self.get_input_port().Eval(context)[
            self._ft_sensor_weldjoint_idx
        ]

        # Invert using action-reaction principle.
        f_CJc_Jc = -F_CJc_Jc.translational()
        tau_CJc_Jc = -F_CJc_Jc.rotational()

        ft_sensor_measurement = np.concatenate([f_CJc_Jc, tau_CJc_Jc])
        output.set_value(ft_sensor_measurement)
