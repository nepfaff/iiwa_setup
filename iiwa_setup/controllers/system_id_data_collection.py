import logging

from enum import Enum

import numpy as np

from pydrake.all import (
    AbstractValue,
    BasicVector,
    Context,
    Diagram,
    DiagramBuilder,
    LeafSystem,
    MultibodyPlant,
    PathParameterizedTrajectory,
    State,
    Trajectory,
)

from iiwa_setup.dataclasses import TrajectoryWithTimingInformation
from iiwa_setup.motion_planning import (
    plan_unconstrained_gcs_path_start_to_goal,
    reparameterize_with_toppra,
)

from .trajectory_sources import TrajectoryWithTimingInformationSource


class SystemIDDataCollectionPlannerState(Enum):
    """FSM states for SystemIDDataCollectionPlanner."""

    PLAN_MOVE_TO_START = 0
    MOVE_TO_START = 1
    WAIT_BEFORE_COLLECTING_DATA = 2
    COLLECT_DATA = 3
    FINISHED = 4


class SystemIDDataCollectionPlanner(LeafSystem):
    """
    Planner for SystemIDDataCollectionController. It uses a simple FSM.
    """

    def __init__(
        self,
        num_joint_positions: int,
        controller_plant: MultibodyPlant,
        system_id_joint_traj: Trajectory,
        move_to_start_velocity_limits: np.ndarray,
        move_to_start_acceleration_limits: np.ndarray,
        wait_before_collecting_data_time_s: float = 1.0,
    ):
        """
        Args:
            num_joint_positions (int): The number of robot joint positions.
            controller_plant (MultibodyPlant): The robot controller plant for
            controlling the robot.
            system_id_joint_traj (Trajectory): The system ID joint trajectory.
            move_to_start_velocity_limits (np.ndarray): The robot velocity limits
            for moving to the starting position. Shape (N,num_joint_positions) where N
            are the number of robot joints.
            move_to_start_acceleration_limits (np.ndarray): The robot acceleration
            limits for moving to the starting position. Shape (N,num_joint_positions)
            where N are the number of robot joints.
            wait_before_collecting_data_time_s (float): The time to wait between moving
            to the starting position and collecting data.
        """
        super().__init__()

        self._is_finished = False
        self._system_id_joint_traj_start_time = np.nan

        # self._num_joint_positions = num_joint_positions
        self._controller_plant = controller_plant
        self._system_id_joint_traj = system_id_joint_traj
        self._move_to_start_velocity_limits = move_to_start_velocity_limits
        self._move_to_start_acceleration_limits = move_to_start_acceleration_limits
        self._wait_before_collecting_data_time_s = wait_before_collecting_data_time_s

        # Internal state
        self._fsm_state_idx = int(
            self.DeclareAbstractState(
                AbstractValue.Make(
                    SystemIDDataCollectionPlannerState.PLAN_MOVE_TO_START
                )
            )
        )
        """The current FSM state."""
        self._current_joint_traj_idx = int(
            self.DeclareAbstractState(
                AbstractValue.Make(TrajectoryWithTimingInformation())
            )
        )
        """The current joint trajectory."""
        self._current_iiwa_positions_idx = int(
            self.DeclareDiscreteState(num_joint_positions)
        )
        """The current iiwa positions. These are used to command the robot to stay idle."""

        # Input ports
        self._iiwa_position_measured_input_port = self.DeclareVectorInputPort(
            "iiwa.position_measured", num_joint_positions
        )

        # Output ports
        self.DeclareAbstractOutputPort(
            "joint_position_trajectory",
            lambda: AbstractValue.Make(TrajectoryWithTimingInformation()),
            self._get_current_joint_position_trajectory,
        )
        self.DeclareVectorOutputPort(
            "current_iiwa_positions",
            num_joint_positions,
            self._get_current_iiwa_positions,
        )

        # Run FSM logic before every trajectory-advancing step
        self.DeclarePerStepUnrestrictedUpdateEvent(self._run_fsm_logic)

    def _get_current_joint_position_trajectory(
        self, context: Context, output: AbstractValue
    ) -> None:
        """Outputs the current joint position trajectory."""
        current_joint_traj = context.get_abstract_state(
            self._current_joint_traj_idx
        ).get_value()
        output.set_value(current_joint_traj)

    def _get_current_iiwa_positions(
        self, context: Context, output: BasicVector
    ) -> None:
        positions = context.get_discrete_state(
            self._current_iiwa_positions_idx
        ).get_value()
        output.set_value(positions)

    def _plan_move_to_start(self, context: Context) -> PathParameterizedTrajectory:
        """
        Plans the joint trajectory for moving from the inital positions to the system ID
        trajectory starting positions. The path is computed using GCS and is
        reparameterized using Toppra.
        NOTE that the GCS planning assumes that there are no obstacles.

        Returns:
            PathParameterizedTrajectory: The joint trajectory from the current positions
            to the pushing start positions.
        """
        q_current: np.ndarray = self._iiwa_position_measured_input_port.Eval(context)
        q_goal = self._system_id_joint_traj.value(0.0).squeeze().T

        traj = plan_unconstrained_gcs_path_start_to_goal(
            plant=self._controller_plant, q_start=q_current, q_goal=q_goal
        )
        if traj is None:
            logging.error(
                "Failed to find a path to the system ID trajectory start positions."
            )
            exit(1)

        toppra_traj = reparameterize_with_toppra(
            trajectory=traj,
            plant=self._controller_plant,
            velocity_limits=self._move_to_start_velocity_limits,
            acceleration_limits=self._move_to_start_acceleration_limits,
        )
        return toppra_traj

    def _run_fsm_logic(self, context: Context, state: State) -> None:
        """FSM state transition logic."""
        mutable_fsm_state = state.get_mutable_abstract_state(self._fsm_state_idx)
        fsm_state_value: SystemIDDataCollectionPlannerState = (
            context.get_abstract_state(self._fsm_state_idx).get_value()
        )

        if fsm_state_value == SystemIDDataCollectionPlannerState.PLAN_MOVE_TO_START:
            logging.info("Current state: PLAN_MOVE_TO_START")
            q_traj = self._plan_move_to_start(context)
            state.get_mutable_abstract_state(self._current_joint_traj_idx).set_value(
                TrajectoryWithTimingInformation(
                    trajectory=q_traj,
                    start_time_s=context.get_time(),
                )
            )

            logging.info("Transitioning to MOVE_TO_START FSM state.")
            mutable_fsm_state.set_value(
                SystemIDDataCollectionPlannerState.MOVE_TO_START
            )

        elif fsm_state_value == SystemIDDataCollectionPlannerState.MOVE_TO_START:
            q_current: np.ndarray = self._iiwa_position_measured_input_port.Eval(
                context
            )
            q_goal = self._system_id_joint_traj.value(0.0).squeeze().T
            if not np.allclose(q_current, q_goal, atol=0.1):
                return

            logging.info("Transitioning to WAIT_BEFORE_COLLECTING_DATA FSM state.")
            mutable_fsm_state.set_value(
                SystemIDDataCollectionPlannerState.WAIT_BEFORE_COLLECTING_DATA
            )

        elif (
            fsm_state_value
            == SystemIDDataCollectionPlannerState.WAIT_BEFORE_COLLECTING_DATA
        ):
            current_time = context.get_time()
            move_to_start_trajectory: TrajectoryWithTimingInformation = (
                state.get_mutable_abstract_state(
                    self._current_joint_traj_idx
                ).get_value()
            )
            if current_time < (
                move_to_start_trajectory.trajectory.end_time()
                + self._wait_before_collecting_data_time_s
            ):
                return

            current_time = context.get_time()
            state.get_mutable_abstract_state(self._current_joint_traj_idx).set_value(
                TrajectoryWithTimingInformation(
                    trajectory=self._system_id_joint_traj,
                    start_time_s=current_time,
                )
            )
            self._system_id_joint_traj_start_time = current_time

            logging.info("Transitioning to COLLECT_DATA FSM state.")
            mutable_fsm_state.set_value(SystemIDDataCollectionPlannerState.COLLECT_DATA)

        elif fsm_state_value == SystemIDDataCollectionPlannerState.COLLECT_DATA:
            q_current: np.ndarray = self._iiwa_position_measured_input_port.Eval(
                context
            )
            q_goal = (
                self._system_id_joint_traj.value(self._system_id_joint_traj.end_time())
                .squeeze()
                .T
            )
            if not np.allclose(q_current, q_goal, atol=0.1):
                return

            logging.info("Transitioning to FINISHED FSM state.")
            mutable_fsm_state.set_value(SystemIDDataCollectionPlannerState.FINISHED)

        elif fsm_state_value == SystemIDDataCollectionPlannerState.FINISHED:
            self._is_finished = True

        else:
            logging.error(f"Invalid FSM state: {fsm_state_value}")
            exit(1)

    def is_finished(self) -> bool:
        """Returns True if the task has been completed and False otherwise."""
        return self._is_finished

    def get_system_id_joint_traj_start_time(self) -> float:
        return self._system_id_joint_traj_start_time


class SystemIDDataCollectionController(Diagram):
    """
    A controller that moves the robot to the start positions of `system_id_joint_traj`
    using a joint-space trajectory (NOTE that the trajectory is computed without
    collision checks). It then executes the provided `system_id_joint_traj`.
    """

    def __init__(
        self,
        controller_plant: MultibodyPlant,
        num_joint_positions: int,
        system_id_joint_traj: Trajectory,
        move_to_start_velocity_limits: np.ndarray,
        move_to_start_acceleration_limits: np.ndarray,
    ):
        """
        Args:
            controller_plant (MultibodyPlant): The robot controller plant for
            controlling the robot.
            num_joint_positions (int): The number of robot joint positions.
            system_id_joint_traj (Trajectory): The system ID joint trajectory.
            move_to_start_velocity_limits (np.ndarray): The robot velocity limits
            for moving to the starting position. Shape (N,num_joint_positions) where N
            are the number of robot joints.
            move_to_start_acceleration_limits (np.ndarray): The robot acceleration
            limits for moving to the starting position. Shape (N,num_joint_positions)
            where N are the number of robot joints.
        """
        super().__init__()

        builder = DiagramBuilder()

        self._planer: SystemIDDataCollectionPlanner = builder.AddNamedSystem(
            "planer",
            SystemIDDataCollectionPlanner(
                num_joint_positions=num_joint_positions,
                controller_plant=controller_plant,
                system_id_joint_traj=system_id_joint_traj,
                move_to_start_velocity_limits=move_to_start_velocity_limits,
                move_to_start_acceleration_limits=move_to_start_acceleration_limits,
            ),
        )
        builder.ExportInput(
            self._planer.GetInputPort("iiwa.position_measured"),
            "iiwa.position_measured",
        )

        joint_traj_source: TrajectoryWithTimingInformationSource = (
            builder.AddNamedSystem(
                "joint_traj_source",
                TrajectoryWithTimingInformationSource(
                    trajectory_size=num_joint_positions
                ),
            )
        )
        builder.Connect(
            self._planer.GetOutputPort("joint_position_trajectory"),
            joint_traj_source.GetInputPort("trajectory"),
        )
        builder.Connect(
            self._planer.GetOutputPort("current_iiwa_positions"),
            joint_traj_source.GetInputPort("current_cmd"),
        )
        builder.ExportOutput(
            joint_traj_source.get_output_port(),
            "iiwa.position",
        )

        builder.BuildInto(self)

    def is_finished(self) -> bool:
        """Returns True if the task has been completed and False otherwise."""
        return self._planer.is_finished()

    def get_system_id_joint_traj_start_time(self) -> float:
        return self._planer.get_system_id_joint_traj_start_time()
