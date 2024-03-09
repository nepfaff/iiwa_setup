import os

from functools import partial
from typing import List, Union

import numpy as np

from manipulation.station import (
    AddIiwa,
    AddPointClouds,
    AddWsg,
    ConfigureParser,
    MakeHardwareStation,
    Scenario,
)
from pydrake.all import (
    AbstractValue,
    BasicVector,
    CollisionFilterDeclaration,
    Context,
    Demultiplexer,
    Diagram,
    DiagramBuilder,
    GeometrySet,
    IiwaControlMode,
    LeafSystem,
    MatrixGain,
    ModelDirectives,
    ModelInstanceIndex,
    MultibodyPlant,
    MultibodyPositionToGeometryPose,
    Multiplexer,
    OutputPort,
    ParseIiwaControlMode,
    Parser,
    ProcessModelDirectives,
    RigidTransform,
    SceneGraph,
    StartMeshcat,
    State,
    position_enabled,
    torque_enabled,
)

from iiwa_setup.util import get_package_xmls


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
            if "iiwa" in model_instance_name:
                # The iiwa state should be obtained from the external station
                continue
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
        control_mode: Union[IiwaControlMode, str] = IiwaControlMode.kPositionOnly,
        create_point_clouds: bool = False,
        package_xmls: List[str] = [],
    ):
        """
        Args:
            scenario (Scenario): The scenario to use. This must contain one iiwa.
            has_wsg (bool): Whether the station has a WSG gripper. This gripper must be
                part of the scenario. If false, then the iiwa controller plant will not
                have a WSG gripper (tracking is less accurate if there is a mismatch).
            use_hardware (bool): Whether to use real world hardware.
            control_mode (Union[IiwaControlMode, str], optional): The control mode to
                use. Must be one of "position_and_torque", "position_only", or
                "torque_only".
            create_point_clouds (bool, optional): Whether to create point clouds from
                the camera images. Defaults to False. Setting this to True might add
                computational overhead.
        """
        super().__init__()

        self._use_hardware = use_hardware
        if isinstance(control_mode, str):
            control_mode = ParseIiwaControlMode(control_mode)

        for package_path in get_package_xmls():
            if os.path.exists(package_path):
                package_xmls.append(package_path)

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
        self._external_station_diagram = MakeHardwareStation(
            scenario=scenario,
            meshcat=self.external_meshcat,
            hardware=use_hardware,
            package_xmls=package_xmls,
        )
        self._external_station_diagram.set_name("external_station")
        self._external_scene_graph = self._external_station_diagram.GetSubsystemByName(
            "scene_graph"
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
        exported_internal_station_port_names = [
            "body_poses",
            "query_object",
        ]
        for port_name in exported_internal_station_port_names:
            builder.ExportOutput(
                self.internal_station.GetOutputPort(port_name), port_name
            )
        internal_plant = self.internal_station.get_plant()
        for i in range(internal_plant.num_model_instances()):
            model_instance = ModelInstanceIndex(i)
            model_instance_name = internal_plant.GetModelInstanceName(model_instance)
            port_name = f"{model_instance_name}_state"
            if self.internal_station.HasOutputPort(port_name):
                builder.ExportOutput(
                    self.internal_station.GetOutputPort(port_name), port_name
                )
                exported_internal_station_port_names.append(port_name)

        # Export external station ports
        for i in range(self._external_station.num_input_ports()):
            port = self._external_station.get_input_port(i)
            name = port.get_name()
            if (
                name not in exported_internal_station_port_names
                and not builder.IsConnectedOrExported(port)
            ):
                builder.ExportInput(port, name)
        for i in range(self._external_station.num_output_ports()):
            port = self._external_station.get_output_port(i)
            name = port.get_name()
            if name not in exported_internal_station_port_names:
                builder.ExportOutput(port, name)
        if (
            len(scenario.cameras.items()) > 0
            and create_point_clouds
            and not use_hardware
        ):
            # TODO: Remove plant dependency from AddPointClouds to allow using it with
            # hardware
            depth_img_to_pcd_systems = AddPointClouds(
                scenario=scenario,
                station=self._external_station,
                builder=builder,
                meshcat=self.external_meshcat,
            )
            for _, camera_config in scenario.cameras.items():
                name = camera_config.name
                builder.ExportOutput(
                    depth_img_to_pcd_systems[name].point_cloud_output_port(),
                    f"{name}.point_cloud",
                )

        builder.BuildInto(self)

    def get_internal_plant(self) -> MultibodyPlant:
        """Get the internal non-simulated plant."""
        return self.internal_station.get_plant()

    def get_internal_plant_context(self) -> Context:
        return self.internal_station.get_plant_context()

    def get_iiwa_controller_plant(self) -> MultibodyPlant:
        return self.internal_station.get_iiwa_controller_plant()

    def get_model_instance(self, name: str) -> ModelInstanceIndex:
        plant = self.get_internal_plant()
        return plant.GetModelInstanceByName(name)

    def exclude_object_from_collision(self, context: Context, object_name: str) -> None:
        """
        Excludes collisions between the object and everything else (uses a collision
        filter) during simulation.
        NOTE: Should only be used when the real world is simulated.

        Args:
            context (Context): The diagram context.
            object_name (str): The name of the object to exclude collisions for.
        """
        if self._use_hardware:
            raise RuntimeError(
                "This method should only be used when the real world is simulated!"
            )

        external_plant: MultibodyPlant = self._external_station.GetSubsystemByName(
            "plant"
        )

        # Get the collision geometries of all the bodies in the plant
        geometry_set_all = GeometrySet()
        for i in range(external_plant.num_model_instances()):
            model_instance = ModelInstanceIndex(i)
            body_indices = external_plant.GetBodyIndices(model_instance)
            bodies = [
                external_plant.get_body(body_index) for body_index in body_indices
            ]
            geometry_ids = [
                external_plant.GetCollisionGeometriesForBody(body) for body in bodies
            ]
            for id in geometry_ids:
                geometry_set_all.Add(id)

        # Get the collision geometries of the object
        object_body = external_plant.GetBodyByName(object_name + "_base_link")
        object_geometry_ids = external_plant.GetCollisionGeometriesForBody(object_body)

        # Exclude collision between the object and everything else
        object_exclude_declaration = CollisionFilterDeclaration().ExcludeBetween(
            GeometrySet(object_geometry_ids), geometry_set_all
        )
        external_scene_graph_context: Context = (
            self._external_scene_graph.GetMyMutableContextFromRoot(context)
        )
        self._external_scene_graph.collision_filter_manager(
            external_scene_graph_context
        ).Apply(object_exclude_declaration)

    def disable_gravity(self) -> None:
        """
        Disables gravity in the simulation.
        NOTE: Should only be used when the real world is simulated.
        """
        if self._use_hardware:
            raise RuntimeError(
                "This method should only be used when the real world is simulated!"
            )

        self._external_station.GetSubsystemByName(
            "plant"
        ).mutable_gravity_field().set_gravity_vector(np.zeros(3))
