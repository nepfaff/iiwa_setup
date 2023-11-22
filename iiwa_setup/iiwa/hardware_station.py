import os

from typing import List, Tuple

import numpy as np

from manipulation.station import (
    ApplyCameraConfigSim,
    ApplyDriverConfigsSim,
    ApplyVisualizationConfig,
    ConfigureParser,
    MakeHardwareStationInterface,
    Scenario,
)
from pydrake.all import (
    AddMultibodyPlant,
    CollisionFilterDeclaration,
    Context,
    Demultiplexer,
    Diagram,
    DiagramBuilder,
    GeometrySet,
    Meshcat,
    ModelDirectives,
    ModelInstanceIndex,
    MultibodyPlant,
    Multiplexer,
    Parser,
    ProcessModelDirectives,
    SceneGraph,
    StartMeshcat,
)


def MakeHardwareStation(
    scenario: Scenario,
    meshcat: Meshcat = None,
    *,
    package_xmls: List[str] = [],
    hardware: bool = False,
) -> Tuple[Diagram, SceneGraph]:
    """
    NOTE: This is a modified version of `MakeHardwareStation` from
    https://github.com/RussTedrake/manipulation.

    If `hardware=False`, (the default) returns a HardwareStation diagram containing:
      - A MultibodyPlant with populated via the directives in `scenario`.
      - A SceneGraph
      - The default Drake visualizers
      - Any robot / sensors drivers specified in the YAML description.

    If `hardware=True`, returns a HardwareStationInterface diagram containing the
    network interfaces to communicate directly with the hardware drivers.

    Args:
        scenario: A Scenario structure, populated using the `load_scenario` method.
        meshcat: If not None, then AddDefaultVisualization will be added to the
        subdiagram using this meshcat instance.
        package_xmls: A list of package.xml file paths that will be passed to the
        parser, using Parser.AddPackageXml().

    Returns:
        A tuple of (hardware_station_diagram, scene_graph):
        - hardware_station_diagram: The diagram representing the hardware station.
        - scene_graph: The scene graph of the hardware station if `hardware=False`
        and `None` if `hardware=True`.
    """
    if hardware:
        return (
            MakeHardwareStationInterface(
                scenario=scenario, meshcat=meshcat, package_xmls=package_xmls
            ),
            None,
        )

    builder = DiagramBuilder()

    # Create the multibody plant and scene graph.
    sim_plant: MultibodyPlant
    sim_plant, scene_graph = AddMultibodyPlant(
        config=scenario.plant_config, builder=builder
    )

    parser = Parser(sim_plant)
    for p in package_xmls:
        parser.package_map().AddPackageXml(p)
    ConfigureParser(parser)

    # Add model directives.
    added_models = ProcessModelDirectives(
        directives=ModelDirectives(directives=scenario.directives),
        parser=parser,
    )

    # Now the plant is complete.
    sim_plant.Finalize()

    # Add drivers.
    ApplyDriverConfigsSim(
        driver_configs=scenario.model_drivers,
        sim_plant=sim_plant,
        models_from_directives=added_models,
        builder=builder,
    )

    # Add scene cameras.
    for _, camera in scenario.cameras.items():
        ApplyCameraConfigSim(config=camera, builder=builder)

    # Add visualization.
    ApplyVisualizationConfig(scenario.visualization, builder, meshcat=meshcat)

    # Export "cheat" ports.
    builder.ExportOutput(scene_graph.get_query_output_port(), "query_object")
    builder.ExportOutput(sim_plant.get_contact_results_output_port(), "contact_results")
    builder.ExportOutput(sim_plant.get_state_output_port(), "plant_continuous_state")
    builder.ExportOutput(sim_plant.get_body_poses_output_port(), "body_poses")
    for i in range(sim_plant.num_model_instances()):
        model_instance = ModelInstanceIndex(i)
        model_instance_name = sim_plant.GetModelInstanceName(model_instance)
        builder.ExportOutput(
            sim_plant.get_state_output_port(model_instance),
            f"{model_instance_name}_state",
        )

    diagram = builder.Build()
    diagram.set_name("station")

    return diagram, scene_graph


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
        self.internal_station_diagram: Diagram
        self.internal_scene_graph: SceneGraph
        internal_station_diagram, self.internal_scene_graph = MakeHardwareStation(
            scenario=scenario,
            meshcat=self.internal_meshcat,
            hardware=False,
            package_xmls=package_xmls,
        )
        self.internal_station: Diagram = builder.AddNamedSystem(
            "internal_station", internal_station_diagram
        )

        # External Station
        self.external_meshcat = StartMeshcat()
        self._external_station_diagram: Diagram
        self._external_scene_graph: SceneGraph
        (
            self._external_station_diagram,
            self._external_scene_graph,
        ) = MakeHardwareStation(
            scenario=scenario,
            meshcat=self.external_meshcat,
            hardware=use_hardware,
            package_xmls=package_xmls,
        )
        self._external_station = builder.AddNamedSystem(
            "external_station",
            self._external_station_diagram,
        )

        # Connect the output of external station to the input of internal station
        builder.Connect(
            self._external_station.GetOutputPort("iiwa.position_commanded"),
            self.internal_station.GetInputPort("iiwa.position"),
        )
        if has_wsg:
            wsg_state_demux: Demultiplexer = builder.AddSystem(Demultiplexer(2, 1))
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
        builder.ExportOutput(
            self.internal_station.GetOutputPort("query_object"), "query_object"
        )
        internal_plant: MultibodyPlant = self.internal_station.GetSubsystemByName(
            "plant"
        )
        for i in range(internal_plant.num_model_instances()):
            model_instance = ModelInstanceIndex(i)
            model_instance_name = internal_plant.GetModelInstanceName(model_instance)
            port_name = f"{model_instance_name}_state"
            builder.ExportOutput(
                self.internal_station.GetOutputPort(port_name), port_name
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
            self._external_station.GetOutputPort("iiwa.position_measured"),
            "iiwa.position_measured",
        )
        builder.ExportOutput(
            self._external_station.GetOutputPort("iiwa.velocity_estimated"),
            "iiwa.velocity_estimated",
        )
        builder.ExportOutput(
            self._external_station.GetOutputPort("iiwa.torque_measured"),
            "iiwa.torque_measured",
        )
        builder.ExportOutput(
            self._external_station.GetOutputPort("iiwa.torque_commanded"),
            "iiwa.torque_commanded",
        )
        # Export external state output
        iiwa_state_mux: Multiplexer = builder.AddSystem(Multiplexer([7, 7]))
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

    def get_plant(self) -> MultibodyPlant:
        return self.internal_station.GetSubsystemByName("plant")

    def get_iiwa_controller_plant(self) -> MultibodyPlant:
        return self.internal_station.GetSubsystemByName(
            "iiwa.controller"
        ).get_multibody_plant_for_control()

    def get_model_instance(self, name: str) -> ModelInstanceIndex:
        plant = self.get_plant()
        return plant.GetModelInstanceByName(name)

    def exclude_object_from_collision(self, context: Context, object_name: str) -> None:
        """Excludes collisions between the object and everything else (uses a collision
        filter).

        Args:
            context (Context): The diagram context.
            object_name (str): The name of the object to exclude collisions for.
        """
        internal_plant = self.get_plant()
        outer_plant: MultibodyPlant = self._external_station.GetSubsystemByName("plant")
        internal_scene_graph_context: Context = (
            self.internal_scene_graph.GetMyMutableContextFromRoot(context)
        )
        external_scene_graph_context: Context = (
            self._external_scene_graph.GetMyMutableContextFromRoot(context)
        )
        for plant, scene_graph, scene_graph_context in [
            (internal_plant, self.internal_scene_graph, internal_scene_graph_context),
            (
                outer_plant,
                self._external_scene_graph,
                external_scene_graph_context,
            ),
        ]:
            # Get the collision geometries of all the bodies in the plant
            geometry_set_all = GeometrySet()
            for i in range(plant.num_model_instances()):
                model_instance = ModelInstanceIndex(i)
                body_indices = plant.GetBodyIndices(model_instance)
                bodies = [plant.get_body(body_index) for body_index in body_indices]
                geometry_ids = [
                    plant.GetCollisionGeometriesForBody(body) for body in bodies
                ]
                for id in geometry_ids:
                    geometry_set_all.Add(id)

            # Get the collision geometries of the object
            object_body = plant.GetBodyByName(object_name + "_base_link")
            object_geometry_ids = plant.GetCollisionGeometriesForBody(object_body)

            # Exclude collision between the object and everything else
            object_exclude_declaration = CollisionFilterDeclaration().ExcludeBetween(
                GeometrySet(object_geometry_ids), geometry_set_all
            )
            scene_graph.collision_filter_manager(scene_graph_context).Apply(
                object_exclude_declaration
            )

    def disable_gravity(self) -> None:
        self.get_plant().mutable_gravity_field().set_gravity_vector(np.zeros(3))
        self._external_station.GetSubsystemByName(
            "plant"
        ).mutable_gravity_field().set_gravity_vector(np.zeros(3))
