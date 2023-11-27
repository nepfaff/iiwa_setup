import os

from functools import partial
from typing import List, Tuple

import numpy as np

from manipulation.station import (
    AddIiwa,
    AddWsg,
    ApplyCameraConfigSim,
    ApplyDriverConfigsSim,
    ApplyVisualizationConfig,
    ConfigureParser,
    MakeHardwareStationInterface,
    Scenario,
)
from pydrake.all import (
    AbstractValue,
    AddMultibodyPlant,
    BasicVector,
    CollisionFilterDeclaration,
    Context,
    Demultiplexer,
    Diagram,
    DiagramBuilder,
    GeometrySet,
    LeafSystem,
    MatrixGain,
    Meshcat,
    ModelDirectives,
    ModelInstanceIndex,
    MultibodyPlant,
    MultibodyPositionToGeometryPose,
    Multiplexer,
    OutputPort,
    Parser,
    ProcessModelDirectives,
    RigidTransform,
    SceneGraph,
    StartMeshcat,
    State,
)


class PlantUpdater(LeafSystem):
    """
    Provides the API for updating and reading a plant context without simulating
    the plant (adding it to the diagram).
    """

    def __init__(self, plant: MultibodyPlant, has_wsg: bool):
        super().__init__()

        self._plant = plant
        self._has_wsg = has_wsg

        self._plant_context = None
        self._iiwa_model_instance_index = plant.GetModelInstanceByName("iiwa")
        if self._has_wsg:
            self._wsg_model_instance_index = plant.GetModelInstanceByName("wsg")

        # Input ports
        self._iiwa_position_input_port = self.DeclareVectorInputPort(
            "iiwa.position",
            self._plant.num_positions(self._iiwa_model_instance_index),
        )
        if self._has_wsg:
            self._wsg_position_input_port = self.DeclareVectorInputPort(
                "wsg.position",
                self._plant.num_positions(self._wsg_model_instance_index),
            )

        # Output ports
        self._position_output_port = self.DeclareVectorOutputPort(
            "position", self._plant.num_positions(), self._get_position
        )
        self._state_output_port = self.DeclareVectorOutputPort(
            "state",
            self._plant.num_positions() + self._plant.num_velocities(),
            self._get_state,
        )
        self._body_poses_output_port = self.DeclareAbstractOutputPort(
            "body_poses",
            lambda: AbstractValue.Make(
                np.array([RigidTransform] * self._plant.num_bodies())
            ),
            self._get_body_poses,
        )
        for i in range(self._plant.num_model_instances()):
            model_instance = ModelInstanceIndex(i)
            model_instance_name = self._plant.GetModelInstanceName(model_instance)
            self.DeclareVectorOutputPort(
                f"{model_instance_name}_state",
                self._plant.num_positions(model_instance)
                + self._plant.num_velocities(model_instance),
                partial(self._get_state, model_instance=model_instance),
            )

        self.DeclarePerStepUnrestrictedUpdateEvent(self._update_plant)

    def _update_plant(self, context: Context, state: State) -> None:
        if self._plant_context is None:
            self._plant_context = self._plant.CreateDefaultContext()

        # Update iiwa positions
        self._plant.SetPositions(
            self._plant_context,
            self._iiwa_model_instance_index,
            self._iiwa_position_input_port.Eval(context),
        )

        if self._has_wsg:
            # Update wsg positions
            self._plant.SetPositions(
                self._plant_context,
                self._wsg_model_instance_index,
                self._wsg_position_input_port.Eval(context),
            )

    def _get_position(self, context: Context, output: BasicVector) -> None:
        if self._plant_context is None:
            self._plant_context = self._plant.CreateDefaultContext()

        positions = self._plant.GetPositions(self._plant_context)
        output.set_value(positions)

    def get_position_output_port(self) -> OutputPort:
        return self._position_output_port

    def _get_state(
        self,
        context: Context,
        output: BasicVector,
        model_instance: ModelInstanceIndex = None,
    ) -> None:
        if self._plant_context is None:
            self._plant_context = self._plant.CreateDefaultContext()

        state = self._plant.GetPositionsAndVelocities(
            self._plant_context, model_instance
        )
        output.set_value(state)

    def get_state_output_port(
        self, model_instance: ModelInstanceIndex = None
    ) -> OutputPort:
        if model_instance is None:
            return self._state_output_port
        model_instance_name = self._plant.GetModelInstanceName(model_instance)
        return self.GetOutputPort(f"{model_instance_name}_state")

    def _get_body_poses(self, context: Context, output: AbstractValue) -> None:
        if self._plant_context is None:
            self._plant_context = self._plant.CreateDefaultContext()

        body_poses = []
        for body_idx in self._plant.GetBodyIndices():
            body = self._plant.get_body(body_idx)
            pose = self._plant.CalcRelativeTransform(
                context=self._plant_context,
                frame_A=self._plant.world_frame(),
                frame_B=body.body_frame(),
            )
            body_poses.append(pose)
        output.set_value(np.array(body_poses))

    def get_body_poses_output_port(self) -> OutputPort:
        return self._body_poses_output_port

    def get_plant_context(self) -> Context:
        if self._plant_context is None:
            self._plant_context = self._plant.CreateDefaultContext()
        return self._plant_context


class InternalStationDiagram(Diagram):
    """
    The "internal" station represents our knowledge of the real world and is not
    simulated. It contains a plant which is updated using a plant updater system. The
    plant itself is not part of the diagram while the updater system is.
    """

    def __init__(
        self,
        scenario: Scenario,
        has_wsg: bool,
        package_xmls: List[str] = [],
    ):
        super().__init__()

        builder = DiagramBuilder()

        # Create the multibody plant and scene graph
        self._plant = MultibodyPlant(time_step=scenario.plant_config.time_step)
        self._plant.set_name("internal_plant")
        self._scene_graph = builder.AddNamedSystem("scene_graph", SceneGraph())
        self._plant.RegisterAsSourceForSceneGraph(self._scene_graph)

        parser = Parser(self._plant)
        for p in package_xmls:
            parser.package_map().AddPackageXml(p)
        ConfigureParser(parser)

        # Add model directives
        _ = ProcessModelDirectives(
            directives=ModelDirectives(directives=scenario.directives),
            parser=parser,
        )

        self._plant.Finalize()

        # Add system for updating the plant
        self._plant_updater: PlantUpdater = builder.AddNamedSystem(
            "plant_updater", PlantUpdater(plant=self._plant, has_wsg=has_wsg)
        )

        # Connect the plant to the scene graph
        mbp_position_to_geometry_pose: MultibodyPositionToGeometryPose = (
            builder.AddNamedSystem(
                "mbp_position_to_geometry_pose",
                MultibodyPositionToGeometryPose(self._plant),
            )
        )
        builder.Connect(
            self._plant_updater.get_position_output_port(),
            mbp_position_to_geometry_pose.get_input_port(),
        )
        builder.Connect(
            mbp_position_to_geometry_pose.get_output_port(),
            self._scene_graph.get_source_pose_port(self._plant.get_source_id()),
        )

        # Make the plant for the iiwa controller
        self._iiwa_controller_plant = MultibodyPlant(time_step=self._plant.time_step())
        controller_iiwa = AddIiwa(self._iiwa_controller_plant)
        if has_wsg:
            AddWsg(self._iiwa_controller_plant, controller_iiwa, welded=True)
        self._iiwa_controller_plant.Finalize()

        # Export input ports
        builder.ExportInput(
            self._plant_updater.GetInputPort("iiwa.position"), "iiwa.position"
        )
        if has_wsg:
            builder.ExportInput(
                self._plant_updater.GetInputPort("wsg.position"), "wsg.position"
            )

        # Export "cheat" ports
        builder.ExportOutput(self._scene_graph.get_query_output_port(), "query_object")
        builder.ExportOutput(
            self._plant_updater.get_state_output_port(), "plant_continuous_state"
        )
        builder.ExportOutput(
            self._plant_updater.get_body_poses_output_port(), "body_poses"
        )
        for i in range(self._plant.num_model_instances()):
            model_instance = ModelInstanceIndex(i)
            model_instance_name = self._plant.GetModelInstanceName(model_instance)
            builder.ExportOutput(
                self._plant_updater.get_state_output_port(model_instance),
                f"{model_instance_name}_state",
            )

        builder.BuildInto(self)

    def get_plant(self) -> MultibodyPlant:
        return self._plant

    def get_plant_context(self) -> Context:
        return self._plant_updater.get_plant_context()

    def get_iiwa_controller_plant(self) -> MultibodyPlant:
        return self._iiwa_controller_plant

    def get_scene_graph(self) -> SceneGraph:
        return self._scene_graph


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

    # Create the multibody plant and scene graph
    sim_plant: MultibodyPlant
    sim_plant, scene_graph = AddMultibodyPlant(
        config=scenario.plant_config, builder=builder
    )

    parser = Parser(sim_plant)
    for p in package_xmls:
        parser.package_map().AddPackageXml(p)
    ConfigureParser(parser)

    # Add model directives
    added_models = ProcessModelDirectives(
        directives=ModelDirectives(directives=scenario.directives),
        parser=parser,
    )

    # Now the plant is complete
    sim_plant.Finalize()

    # Add drivers
    ApplyDriverConfigsSim(
        driver_configs=scenario.model_drivers,
        sim_plant=sim_plant,
        models_from_directives=added_models,
        builder=builder,
    )

    # Add scene cameras
    for _, camera in scenario.cameras.items():
        ApplyCameraConfigSim(config=camera, builder=builder)

    # Add visualization
    ApplyVisualizationConfig(scenario.visualization, builder, meshcat=meshcat)

    # Export "cheat" ports
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
    diagram.set_name("external_station")

    return diagram, scene_graph


class IiwaHardwareStationDiagram(Diagram):
    """
    Consists of an "internal" and and "external" hardware station. The "external"
    station represents the real world or simulated version of it. The "internal" station
    represents our knowledge of the real world and is not simulated.
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
        self.internal_station: InternalStationDiagram = builder.AddNamedSystem(
            "internal_station",
            InternalStationDiagram(
                scenario=scenario,
                has_wsg=has_wsg,
                package_xmls=package_xmls,
            ),
        )
        self.internal_scene_graph = self.internal_station.get_scene_graph()

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
        self._external_station: Diagram = builder.AddNamedSystem(
            "external_station",
            self._external_station_diagram,
        )

        # Connect the output of external station to the input of internal station
        # NOTE: Measured and commanded positions can be quite different
        builder.Connect(
            # self._external_station.GetOutputPort("iiwa.position_commanded"),
            self._external_station.GetOutputPort("iiwa.position_measured"),
            self.internal_station.GetInputPort("iiwa.position"),
        )
        if has_wsg:
            wsg_state_demux: Demultiplexer = builder.AddSystem(Demultiplexer(2, 1))
            builder.Connect(
                self._external_station.GetOutputPort("wsg.state_measured"),
                wsg_state_demux.get_input_port(),
            )
            # System for converting the distance between the fingers to the positions of
            # the two finger joints
            wsg_state_to_wsg_mbp_state = builder.AddNamedSystem(
                "wsg_state_to_wsg_mbp_state", MatrixGain(np.array([-0.5, 0.5]))
            )
            builder.Connect(
                wsg_state_demux.get_output_port(0),
                wsg_state_to_wsg_mbp_state.get_input_port(),
            )
            builder.Connect(
                wsg_state_to_wsg_mbp_state.get_output_port(),
                self.internal_station.GetInputPort("wsg.position"),
            )

        # Export internal station ports
        builder.ExportOutput(
            self.internal_station.GetOutputPort("body_poses"), "body_poses"
        )
        builder.ExportOutput(
            self.internal_station.GetOutputPort("query_object"), "query_object"
        )
        internal_plant = self.internal_station.get_plant()
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
        return self.internal_station.get_plant()

    def get_plant_context(self) -> Context:
        return self.internal_station.get_plant_context()

    def get_iiwa_controller_plant(self) -> MultibodyPlant:
        return self.internal_station.get_iiwa_controller_plant()

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
