from copy import copy
from typing import List

import numpy as np

from optitrack import optitrack_frame_t, optitrack_rigid_body_t
from pydrake.all import (
    AbstractValue,
    BasicVector,
    Context,
    Diagram,
    DiagramBuilder,
    DrakeLcm,
    LcmInterfaceSystem,
    LcmSubscriberSystem,
    LeafSystem,
    ModelInstanceIndex,
    MultibodyPlant,
    Quaternion,
    RigidTransform,
    RollPitchYaw,
    RotationMatrix,
    State,
)

from iiwa_setup.iiwa import IiwaHardwareStationDiagram


class OptitrackObjectTransformUpdater(LeafSystem):
    """
    A system that updates the pose of `object_instance_idx` in the `plant` before
    every trajectory-advancing step using the optitrack measurements.

    NOTE: `set_plant_context` must be used to set the plant's context before the
    simulation is started. The plant context must be obtained from the context that
    is passes to the simulator.
    """

    def __init__(
        self,
        plant: MultibodyPlant,
        object_instance_idx: ModelInstanceIndex,
        optitrack_iiwa_id: int,
        optitrack_body_id: int,
        X_optitrackBody_plantBody_world: RigidTransform,
        retain_z: bool,
        retain_roll: bool,
        retain_pitch: bool,
        X_world_iiwa: RigidTransform = RigidTransform.Identity(),
    ):
        """
        Args:
            plant (MultibodyPlant): The plant that contains the object whose pose should
            be set based on the optitrack measurements.
            object_instance_idx (ModelInstanceIndex): The model instance index of the
            object in the plant whose positions should be set.
            optitrack_iiwa_id (int): The optrirack body ID that corresponds to the iiwa
            base.
            optitrack_body_id (int): The optitrack body ID that corresponds to the
            object.
            X_optitrackBody_plantBody (RigidTransform): The trasform from the optitrack
            pose to the plant/ simulated pose, expressed in the world frame.
            retain_z (bool): Whether to keep the current z position of the object rather
            than changing it to the optitrack measurement. This can be useful for planar
            pushing tasks, where we know that z doesn't change and small measurement
            errors can lead to the object falling through objects.
            retain_roll (bool): Similar to `retain_z`.
            retain_pitch (bool): Similar to `retain_z`.
            X_world_iiwa (RigidTransform): The pose of the iiwa base with respect ot the
            world frame.
        """
        super().__init__()

        self._plant = plant
        self._plant_context = None
        self._object_instance_idx = object_instance_idx
        self._optitrack_iiwa_id = optitrack_iiwa_id
        self._optitrack_body_id = optitrack_body_id
        self._X_optitrackBody_plantBody_world = X_optitrackBody_plantBody_world
        self._retain_z = retain_z
        self._retain_roll = retain_roll
        self._retain_pitch = retain_pitch
        self._X_world_iiwa = X_world_iiwa

        self._object_positions = np.array([np.nan] * 7)

        self._optitrack_frame_index = self.DeclareAbstractInputPort(
            "optitrack_frame", AbstractValue.Make(optitrack_frame_t)
        ).get_index()

        self.DeclareVectorOutputPort(
            "object_positions", 7, self._get_current_object_positions
        )

        # Update object pose before every trajectory-advancing step
        self.DeclarePerStepUnrestrictedUpdateEvent(self._update_object_pose)

    def set_plant_context(self, context: Context) -> None:
        """
        Sets the plant context. NOTE: The context must be set before simulation is
        started.
        """
        self._plant_context = context

    def _update_object_pose(self, context: Context, state: State) -> None:
        """Update the object pose in the plant based on the optitrack measurements."""
        assert (
            self._plant_context is not None
        ), "The plant context must be set before the simulation is started!"

        # Get optitrack measurement
        optitrack_frame: optitrack_frame_t = self.get_input_port(
            self._optitrack_frame_index
        ).Eval(context)
        optitrack_rigid_bodies: List[
            optitrack_rigid_body_t
        ] = optitrack_frame.rigid_bodies

        optitrack_body_ids = [body.id for body in optitrack_rigid_bodies]
        iiwa_base_body = optitrack_rigid_bodies[optitrack_body_ids.index(4)]
        object_body = optitrack_rigid_bodies[
            optitrack_body_ids.index(self._optitrack_body_id)
        ]

        # NOTE: The 'origin' frame refers to the optitrack world frame while the 'world'
        # frame refers to the plant/ simulated world frame that is of actual interest
        X_origin_iiwa = RigidTransform(
            OptitrackObjectTransformUpdater.get_quaternion_from_optitrack_rigid_body(
                iiwa_base_body
            ),
            iiwa_base_body.xyz,
        )
        X_iiwa_origin = X_origin_iiwa.inverse()
        X_origin_optitrackBody = RigidTransform(
            RotationMatrix(self.get_quaternion_from_optitrack_rigid_body(object_body)),
            object_body.xyz,
        )

        # Find the body pose in the plant world frame
        X_world_optitrackBody: RigidTransform = (
            self._X_world_iiwa @ X_iiwa_origin @ X_origin_optitrackBody
        )
        X_optitrackBody_plantBody = RigidTransform(
            self._X_optitrackBody_plantBody_world.rotation(),
            X_world_optitrackBody.inverse().rotation()
            @ self._X_optitrackBody_plantBody_world.translation(),
        )
        X_world_plantBody: RigidTransform = (
            X_world_optitrackBody @ X_optitrackBody_plantBody
        )
        object_quaternion = copy(X_world_plantBody.rotation().ToQuaternion())
        object_translation = copy(X_world_plantBody.translation())

        # Optionally only ubdate a subset of the positions
        current_object_positions = self._plant.GetPositions(
            self._plant_context, self._object_instance_idx
        )
        if self._retain_z:
            object_translation[2] = current_object_positions[-1]
        # Creating a Quaternion might throw an error if the input is not perfectly
        # normalized
        current_object_RPY = (
            RotationMatrix(
                Quaternion(
                    current_object_positions[:4]
                    / np.linalg.norm(current_object_positions[:4])
                )
            )
            .ToRollPitchYaw()
            .vector()
        )
        object_RPY = RotationMatrix(object_quaternion).ToRollPitchYaw().vector()
        if self._retain_roll:
            object_RPY[0] = current_object_RPY[0]
        if self._retain_pitch:
            object_RPY[1] = current_object_RPY[1]
        object_quaternion = RotationMatrix(RollPitchYaw(object_RPY)).ToQuaternion()

        new_object_positions = np.concatenate(
            (object_quaternion.wxyz(), object_translation)
        )
        self._plant.SetPositions(
            self._plant_context, self._object_instance_idx, new_object_positions
        )
        self._object_positions = new_object_positions

    def _get_current_object_positions(
        self, context: Context, output: BasicVector
    ) -> None:
        output.set_value(self._object_positions)

    @staticmethod
    def get_quaternion_from_optitrack_rigid_body(body: optitrack_rigid_body_t):
        return Quaternion(
            x=body.quat[0], y=body.quat[1], z=body.quat[2], w=body.quat[3]
        )


class OptitrackObjectTransformUpdaterDiagram(Diagram):
    """
    A diagram for updating the pose of `object_instance_idx` in the `plant` before
    every trajectory-advancing step using the optitrack measurements.

    NOTE: `set_plant_context` must be used to set the plant's context before the
    simulation is started. The plant context must be obtained from the context that
    is passes to the simulator.
    """

    def __init__(
        self,
        station: IiwaHardwareStationDiagram,
        object_name: str,
        optitrack_iiwa_id: int,
        optitrack_body_id: int,
        X_optitrackBody_plantBody_world: RigidTransform,
        retain_z: bool = False,
        retain_roll: bool = False,
        retain_pitch: bool = False,
        X_world_iiwa: RigidTransform = RigidTransform.Identity(),
    ):
        """
        Args:
            station (IiwaHardwareStationDiagram): The iiwa hardware station.
            optitrack_iiwa_id (int): The optrirack body ID that corresponds to the iiwa
            base.
            object_name (str): The name of the object in the plant whose positions
            should be set.
            optitrack_body_id (int): The optitrack body ID that corresponds to the
            object.
            X_optitrackBody_plantBody (RigidTransform): The trasform from the optitrack
            pose to the plant/ simulated pose, expressed in the world frame.
            retain_z (bool): Whether to keep the current z position of the object rather
            than changing it to the optitrack measurement. This can be useful for planar
            pushing tasks, where we know that z doesn't change and small measurement
            errors can lead to the object falling through objects.
            retain_roll (bool): Similar to `retain_z`.
            retain_pitch (bool): Similar to `retain_z`.
            X_world_iiwa (RigidTransform): The pose of the iiwa base with respect ot the
            world frame.
        """
        super().__init__()

        builder = DiagramBuilder()

        lcm = DrakeLcm()
        lcm_system = builder.AddSystem(LcmInterfaceSystem(lcm))
        plant = station.get_plant()
        manipuland_instance = station.get_model_instance(object_name)
        self._optitrack_object_transform_updater: OptitrackObjectTransformUpdater = (
            builder.AddNamedSystem(
                "OptitrackObjectTransformUpdater",
                OptitrackObjectTransformUpdater(
                    plant=plant,
                    object_instance_idx=manipuland_instance,
                    optitrack_iiwa_id=optitrack_iiwa_id,
                    optitrack_body_id=optitrack_body_id,
                    X_optitrackBody_plantBody_world=X_optitrackBody_plantBody_world,
                    retain_z=retain_z,
                    retain_roll=retain_roll,
                    retain_pitch=retain_pitch,
                    X_world_iiwa=X_world_iiwa,
                ),
            )
        )
        builder.ExportOutput(
            self._optitrack_object_transform_updater.GetOutputPort("object_positions"),
            "object_positions",
        )

        optitrack_frame_subscriber: LcmSubscriberSystem = builder.AddNamedSystem(
            "OptitrackFrameSubscriber",
            LcmSubscriberSystem.Make(
                channel="OPTITRACK_FRAMES",
                lcm_type=optitrack_frame_t,
                lcm=lcm_system,
                use_cpp_serializer=False,
                wait_for_message_on_initialization_timeout=10,
            ),
        )
        builder.Connect(
            optitrack_frame_subscriber.get_output_port(),
            self._optitrack_object_transform_updater.get_input_port(),
        )

        builder.BuildInto(self)

    def set_plant_context(self, context: Context) -> None:
        """
        Sets the plant context. NOTE: The context must be set before simulation is
        started.
        """
        self._optitrack_object_transform_updater.set_plant_context(context)
