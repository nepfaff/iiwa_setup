"""Script for recording F/T data using the FT 300-S sensor. See README for the
FT 300-S driver installation instructions."""

import argparse
import logging

from pathlib import Path

import numpy as np

from ft_reading_t import ft_reading_t
from pydrake.all import (
    DiagramBuilder,
    DrakeLcm,
    LcmInterfaceSystem,
    LcmSubscriberSystem,
    Simulator,
    VectorLogSink,
)

from iiwa_setup.sensors import FTSensorDataReceiver


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ft_data_output_path",
        type=Path,
        default=None,
        help="The path to save the F/T data to. No data is saved if no path is "
        + "provided.",
    )
    parser.add_argument(
        "--log_level",
        type=str,
        default="INFO",
        choices=["CRITICAL", "ERROR", "WARNING", "INFO", "DEBUG"],
        help="Log level.",
    )
    args = parser.parse_args()
    ft_data_output_path = args.ft_data_output_path

    logging.basicConfig(level=args.log_level)

    builder = DiagramBuilder()

    lcm = DrakeLcm()
    lcm_system = builder.AddSystem(LcmInterfaceSystem(lcm))
    ft_sensor_subscriber: LcmSubscriberSystem = builder.AddNamedSystem(
        "ft_sensor_subscriber",
        LcmSubscriberSystem.Make(
            channel="FT300S_SENSOR_DATA",
            lcm_type=ft_reading_t,
            lcm=lcm_system,
            use_cpp_serializer=False,
            wait_for_message_on_initialization_timeout=10,
        ),
    )
    ft_sensor_data_receiver: FTSensorDataReceiver = builder.AddNamedSystem(
        "ft_sensor_data_receiver", FTSensorDataReceiver()
    )
    builder.Connect(
        ft_sensor_subscriber.get_output_port(), ft_sensor_data_receiver.get_input_port()
    )

    # The FT 300-S sensor publishes at 100Hz.
    ft_logger: VectorLogSink = builder.AddNamedSystem(
        "ft_logger", VectorLogSink(input_size=6, publish_period=1e-2)
    )
    builder.Connect(
        ft_sensor_data_receiver.GetOutputPort("ft_measured"),
        ft_logger.get_input_port(),
    )

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    simulator.Initialize()

    logging.info("Entering infinite publish loop. Exit via KeyboardInterrupt.")
    try:
        while True:
            simulator.AdvanceTo(simulator.get_context().get_time() + 0.1)
    except KeyboardInterrupt:
        pass

    ft_data = ft_logger.FindLog(simulator.get_context()).data().T
    sample_times_s = ft_logger.FindLog(simulator.get_context()).sample_times()

    if ft_data_output_path is not None:
        logging.info(f"Saving F/T data to {ft_data_output_path}.")
        ft_data_output_path.mkdir(parents=True, exist_ok=True)
        np.save(ft_data_output_path / "ft_data.npy", ft_data)
        np.save(ft_data_output_path / "sample_times_s.npy", sample_times_s)


if __name__ == "__main__":
    main()
