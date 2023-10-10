from copy import copy
from typing import List

import numpy as np

from optitrack import optitrack_frame_t, optitrack_rigid_body_t
from pydrake.all import (
    AbstractValue,
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
        retain_z: bool,
        X_optitrackBody_plantBody: RigidTransform,
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
            retain_z (bool): Whether to keep the current z position of the object rather
            than changing it to the optitrack measurement. This can be useful for planar
            pushing tasks, where we know that z doesn't change and small measurement
            errors can lead to the object falling through objects.
            X_optitrackBody_plantBody (RigidTransform): The trasform from the optitrack
            pose to the plant/ simulated pose.
            X_world_iiwa (RigidTransform): The pose of the iiwa base with respect ot the
            world frame.
        """
        super().__init__()

        self._plant = plant
        self._plant_context = None
        self._object_instance_idx = object_instance_idx
        self._optitrack_iiwa_id = optitrack_iiwa_id
        self._optitrack_body_id = optitrack_body_id
        self._retain_z = retain_z
        self._X_optitrackBody_plantBody = X_optitrackBody_plantBody
        self._X_world_iiwa = X_world_iiwa

        self._optitrack_frame_index = self.DeclareAbstractInputPort(
            "optitrack_frame", AbstractValue.Make(optitrack_frame_t)
        ).get_index()

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

        X_world_plantBody = (
            self._X_world_iiwa
            @ X_iiwa_origin
            @ X_origin_optitrackBody
            @ self._X_optitrackBody_plantBody
        )
        object_quaternion = X_world_plantBody.rotation().ToQuaternion().wxyz()
        object_translation = copy(X_world_plantBody.translation())
        if self._retain_z:
            current_object_positions = self._plant.GetPositions(
                self._plant_context, self._object_instance_idx
            )
            object_translation[2] = current_object_positions[6]
        object_pose = np.concatenate((object_quaternion, object_translation))

        self._plant.SetPositions(
            self._plant_context, self._object_instance_idx, object_pose
        )

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
        retain_z: bool,
        X_optitrackBody_plantBody: RigidTransform,
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
            retain_z (bool): Whether to keep the current z position of the object rather
            than changing it to the optitrack measurement. This can be useful for planar
            pushing tasks, where we know that z doesn't change and small measurement
            errors can lead to the object falling through objects.
            X_optitrackBody_plantBody (RigidTransform): The trasform from the optitrack
            pose to the plant/ simulated pose.
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
                    retain_z=retain_z,
                    X_optitrackBody_plantBody=X_optitrackBody_plantBody,
                    X_world_iiwa=X_world_iiwa,
                ),
            )
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
