import logging

try:
    from ft_reading_t import ft_reading_t
except ImportError:
    logging.warning("ft_reading_t module not found. Skipping import.")

from pydrake.all import AbstractValue, BasicVector, Context, LeafSystem


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
