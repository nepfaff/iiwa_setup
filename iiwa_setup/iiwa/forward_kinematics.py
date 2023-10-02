from pydrake.all import (
    AbstractValue,
    Context,
    LeafSystem,
    MultibodyPlant,
    RigidTransform,
)


def forward_kinematics(plant: MultibodyPlant, plant_context: Context) -> RigidTransform:
    """Computes the pose of the iiwa link 7 based on the joint positions stored in the
    context.

    Args:
        plant (MultibodyPlant): The plant that contains the iiwa.
        plant_context (Context): The context that contains the joint positions of the
        iiwa.

    Returns:
        RigidTransform: The pose of the iiwa link 7 in the world frame.
    """
    link_7 = plant.GetBodyByName("iiwa_link_7")
    gripper_frame = link_7.body_frame()
    X_WG = gripper_frame.CalcPoseInWorld(plant_context)
    return X_WG


class IiwaForwardKinematics(LeafSystem):
    """
    A system that takes the iiwa positions as input and outputs the pose of the iiwa
    link 7 in the world frame.
    """

    def __init__(self, plant: MultibodyPlant):
        super().__init__()
        self._plant = plant

        self.DeclareVectorInputPort("iiwa_positions", 7)
        self.DeclareAbstractOutputPort(
            "pose", lambda: AbstractValue.Make(RigidTransform()), self._calc_output
        )

    def _calc_output(self, context: Context, output: AbstractValue) -> None:
        iiwa_positions = self.get_input_port().Eval(context)
        plant_context = self._plant.CreateDefaultContext()
        self._plant.SetPositions(
            plant_context, self._plant.GetModelInstanceByName("iiwa"), iiwa_positions
        )
        X_WG = forward_kinematics(self._plant, plant_context)
        output.get_mutable_value().set(X_WG.rotation(), X_WG.translation())
        output.get_mutable_value().set(X_WG.rotation(), X_WG.translation())
