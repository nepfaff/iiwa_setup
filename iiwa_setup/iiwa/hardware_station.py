import os

from manipulation.station import MakeHardwareStation, Scenario
from pydrake.all import (
    Demultiplexer,
    Diagram,
    DiagramBuilder,
    Multiplexer,
    StartMeshcat,
)


class IiwaHardwareStationDiagram(Diagram):
    """
    Consists of an "internal" and and "external" hardware station. The "internal"
    station mirrors the "external" station and is always simulated. One can think of
    the "internal" station as the internal system model. The "external" station
    represents the hardware or a simulated version of it. Having two stations is
    important as only the "internal" station will contain a plant, etc. when the
    "external" station represents hardware and thus only contains LCM logic.
    """

    def __init__(
        self,
        scenario: Scenario,
        has_wsg: bool,
        use_hardware: bool,
    ):
        super().__init__()
        package_xmls = [
            os.path.join(os.path.dirname(__file__), "../../models/package.xml")
        ]

        builder = DiagramBuilder()

        # Internal Station
        self.internal_meshcat = StartMeshcat()
        self.internal_station = builder.AddNamedSystem(
            "internal_station",
            MakeHardwareStation(
                scenario=scenario,
                meshcat=self.internal_meshcat,
                hardware=False,
                package_xmls=package_xmls,
            ),
        )

        # External Station
        self.external_meshcat = StartMeshcat()
        self._external_station = builder.AddNamedSystem(
            "external_station",
            MakeHardwareStation(
                scenario=scenario,
                meshcat=self.external_meshcat,
                hardware=use_hardware,
                package_xmls=package_xmls,
            ),
        )

        # Connect the output of external station to the input of internal station
        builder.Connect(
            self._external_station.GetOutputPort("iiwa.position_commanded"),
            self.internal_station.GetInputPort("iiwa.position"),
        )
        if has_wsg:
            wsg_state_demux = builder.AddSystem(Demultiplexer(2, 1))
            builder.Connect(
                self._external_station.GetOutputPort("wsg.state_measured"),
                wsg_state_demux.get_input_port(),
            )
            builder.Connect(
                wsg_state_demux.get_output_port(0),
                self.internal_station.GetInputPort("wsg.position"),
            )

        # Export internal station ports
        builder.ExportOutput(
            self.internal_station.GetOutputPort("body_poses"), "body_poses"
        )
        # Export external station ports
        builder.ExportInput(
            self._external_station.GetInputPort("iiwa.position"), "iiwa.position"
        )
        if has_wsg:
            builder.ExportInput(
                self._external_station.GetInputPort("wsg.position"), "wsg.position"
            )
        builder.ExportOutput(
            self._external_station.GetOutputPort("iiwa.position_commanded"),
            "iiwa.position_commanded",
        )
        builder.ExportOutput(
            self._external_station.GetOutputPort("iiwa.velocity_estimated"),
            "iiwa.velocity_estimated",
        )
        # Export external state output
        iiwa_state_mux = builder.AddSystem(Multiplexer([7, 7]))
        builder.Connect(
            self._external_station.GetOutputPort("iiwa.position_measured"),
            iiwa_state_mux.get_input_port(0),
        )
        builder.Connect(
            self._external_station.GetOutputPort("iiwa.velocity_estimated"),
            iiwa_state_mux.get_input_port(1),
        )
        builder.ExportOutput(iiwa_state_mux.get_output_port(), "iiwa.state_estimated")

        builder.BuildInto(self)
