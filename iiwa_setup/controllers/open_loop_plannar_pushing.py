import logging

from dataclasses import dataclass
from enum import Enum

import numpy as np

from pydrake.all import (
    AbstractValue,
    BasicVector,
    Context,
    Diagram,
    DiagramBuilder,
    DiscreteValues,
    InputPortIndex,
    LeafSystem,
    MultibodyPlant,
    PathParameterizedTrajectory,
    PiecewisePose,
    PortSwitch,
    RigidTransform,
    State,
)

from iiwa_setup.dataclasses import (
    PiecewisePoseWithTimingInformation,
    TrajectoryWithTimingInformation,
)
from iiwa_setup.motion_planning import (
    plan_unconstrained_gcs_path_start_to_goal,
    reparameterize_with_toppra,
    solve_global_inverse_kinematics,
)

from .diff_ik_path_following import DiffIKPathFollowingController, DiffIKReseter
from .trajectory_sources import TrajectoryWithTimingInformationSource


class OpenLoopPlanarPushingPlannerState(Enum):
    """FSM states for OpenLoopPlanarPushingPlanner."""

    PLAN_MOVE_TO_START = 0
    MOVE_TO_START = 1
    PLAN_PUSH = 2
    PUSH = 3
    FINISHED = 4


@dataclass
class OpenLoopPlanarPushingPlanarTimingInformation:
    """
    Timing information for OpenLoopPlanarPushingPlanar. All times are relative to the
    simulation start and in seconds. All values are NaN by default.
    """

    start_move_to_start: float = np.nan
    """Start time of the move to pushing start trajectory."""
    end_move_to_start: float = np.nan
    """Finish time of the move to pushing start trajeectory."""
    start_pushing: float = np.nan
    """Start time of the pushing trajectory."""
    end_pushing: float = np.nan
    """End time of the pushing trajectory."""


class OpenLoopPlanarPushingPlanner(LeafSystem):
    """
    Planner for OpenLoopPlanarPushingController. It uses a simple FSM.
    """

    def __init__(
        self,
        num_joint_positions: int,
        controller_plant: MultibodyPlant,
        pushing_start_pose: RigidTransform,
        pushing_pose_trajectory: PiecewisePose,
        gripper_frame_name: str,
        initial_delay_s: float,
        wait_push_delay_s: float,
        move_to_start_velocity_limits: np.ndarray,
        move_to_start_acceleration_limits: np.ndarray,
    ):
        """
        Args:
            num_joint_positions (int): The number of robot joint positions.
            controller_plant (MultibodyPlant): The robot controller plant for
            controlling the robot.
            pushing_start_pose (RigidTransform): The pose that the robot should move to
            before executing the pushing trajectory.
            pushing_pose_trajectory (PiecewisePose): The pushing trajectory that will
            be executed using a differential inverse kinematics controller.
            gripper_frame_name (str): The name of the gripper frame that both the
            `pushing_start_pose` and the `pushing_pose_trajectory` poses refer to.
            initial_delay_s (float): The initial delay before starting a control action.
            wait_push_delay_s (float): The delay between moving to the
            `pushing_start_pose` and executing the `pushing_pose_trajectory`.
            move_to_start_velocity_limits (np.ndarray): The robot velocity limits
            for moving to the starting position. Shape (N,num_joint_positions) where N
            are the number of robot joints.
            move_to_start_acceleration_limits (np.ndarray): The robot acceleration
            limits for moving to the starting position. Shape (N,num_joint_positions)
            where N are the number of robot joints.
        """
        super().__init__()

        self._is_finished = False

        self._num_joint_positions = num_joint_positions
        self._iiwa_controller_plant = controller_plant
        self._pushing_start_pose = pushing_start_pose
        self._gripper_frame_name = gripper_frame_name
        self._pushing_pose_trajectory = pushing_pose_trajectory
        self._wait_push_delay_s = wait_push_delay_s
        self._move_to_start_velocity_limits = move_to_start_velocity_limits
        self._move_to_start_acceleration_limits = move_to_start_acceleration_limits

        # Internal state
        self._fsm_state_idx = int(
            self.DeclareAbstractState(
                AbstractValue.Make(OpenLoopPlanarPushingPlannerState.PLAN_MOVE_TO_START)
            )
        )
        """The current FSM state."""
        self._timing_information_idx = int(
            self.DeclareAbstractState(
                AbstractValue.Make(
                    OpenLoopPlanarPushingPlanarTimingInformation(
                        start_move_to_start=initial_delay_s
                    )
                )
            )
        )
        """Class for writing and reading planed timings."""
        self._current_joint_traj_idx = int(
            self.DeclareAbstractState(
                AbstractValue.Make(TrajectoryWithTimingInformation())
            )
        )
        """The current joint trajectory."""
        self._current_pose_trajectory_idx = int(
            self.DeclareAbstractState(
                AbstractValue.Make(PiecewisePoseWithTimingInformation())
            )
        )
        """The current pose trajectory."""
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
            "control_mode",
            lambda: AbstractValue.Make(InputPortIndex(0)),
            self._calc_control_mode,
        )
        self.DeclareAbstractOutputPort(
            "reset_diff_ik",
            lambda: AbstractValue.Make(False),
            self._calc_diff_ik_reset,
        )
        self.DeclareAbstractOutputPort(
            "joint_position_trajectory",
            lambda: AbstractValue.Make(TrajectoryWithTimingInformation()),
            self._get_current_joint_position_trajectory,
        )
        self.DeclareAbstractOutputPort(
            "pose_trajectory",
            lambda: AbstractValue.Make(PiecewisePoseWithTimingInformation()),
            self._get_current_pose_trajectory,
        )
        self.DeclareVectorOutputPort(
            "current_iiwa_positions",
            num_joint_positions,
            self._get_current_iiwa_positions,
        )

        self.DeclareInitializationDiscreteUpdateEvent(self._initialize_discrete_state)
        # Run FSM logic before every trajectory-advancing step
        self.DeclarePerStepUnrestrictedUpdateEvent(self._run_fsm_logic)

    def _calc_control_mode(self, context: Context, output: AbstractValue) -> None:
        """
        Switches between joint space trajectory following and task space differential
        inverse kinematics.
        """
        state = context.get_abstract_state(int(self._fsm_state_idx)).get_value()
        if state in [
            OpenLoopPlanarPushingPlannerState.PUSH,
            OpenLoopPlanarPushingPlannerState.FINISHED,
        ]:
            # Pose path
            output.set_value(InputPortIndex(2))
        else:
            # Joint position path
            output.set_value(InputPortIndex(1))

    def _calc_diff_ik_reset(self, context: Context, output: AbstractValue) -> None:
        """Logic for deciding when to reset the differential IK controller state."""
        state = context.get_abstract_state(int(self._fsm_state_idx)).get_value()
        if state == OpenLoopPlanarPushingPlannerState.PLAN_PUSH:
            # Need to reset before pushing
            output.set_value(True)
        else:
            output.set_value(False)

    def _get_current_joint_position_trajectory(
        self, context: Context, output: AbstractValue
    ) -> None:
        """Outputs the current joint position trajectory."""
        current_joint_traj = context.get_abstract_state(
            self._current_joint_traj_idx
        ).get_value()
        output.set_value(current_joint_traj)

    def _get_current_pose_trajectory(
        self, context: Context, output: AbstractValue
    ) -> None:
        """Outputs the current pose trajectory."""
        current_pose_traj = context.get_abstract_state(
            self._current_pose_trajectory_idx
        ).get_value()
        output.set_value(current_pose_traj)

    def _get_current_iiwa_positions(
        self, context: Context, output: BasicVector
    ) -> None:
        positions = context.get_discrete_state(
            self._current_iiwa_positions_idx
        ).get_value()
        output.set_value(positions)

    def _initialize_discrete_state(
        self, context: Context, discrete_values: DiscreteValues
    ) -> None:
        # Initialize the current iiwa positions
        discrete_values.set_value(
            self._current_iiwa_positions_idx,
            self._iiwa_position_measured_input_port.Eval(context),
        )

    def _plan_move_to_start(self, context: Context) -> PathParameterizedTrajectory:
        """
        Plans the joint trajectory for moving from the inital pose to the pushing start
        pose. The path is computed using GCS and is reparameterized using Toppra.
        NOTE that the GCS planning assumes that there are no obstacles.

        Returns:
            PathParameterizedTrajectory: The joint trajectory from the current positions
            to the pushing start positions.
        """
        q_current: np.ndarray = self._iiwa_position_measured_input_port.Eval(context)
        q_goal = solve_global_inverse_kinematics(
            plant=self._iiwa_controller_plant,
            X_G=self._pushing_start_pose,
            initial_guess=q_current,
            position_tolerance=0.0,
            orientation_tolerance=0.0,
            gripper_frame_name=self._gripper_frame_name,
        )
        if q_goal is None:
            logging.error(
                "Failed to solve inverse kinematics for the pushing start pose."
            )
            exit(1)

        traj = plan_unconstrained_gcs_path_start_to_goal(
            plant=self._iiwa_controller_plant, q_start=q_current, q_goal=q_goal
        )
        if traj is None:
            logging.error("Failed to find a path to the pushing start positions.")
            exit(1)

        toppra_traj = reparameterize_with_toppra(
            trajectory=traj,
            plant=self._iiwa_controller_plant,
            velocity_limits=self._move_to_start_velocity_limits,
            acceleration_limits=self._move_to_start_acceleration_limits,
        )
        return toppra_traj

    def _run_fsm_logic(self, context: Context, state: State) -> None:
        """FSM state transition logic."""
        current_time = context.get_time()
        timing_information: OpenLoopPlanarPushingPlanarTimingInformation = (
            context.get_mutable_abstract_state(self._timing_information_idx).get_value()
        )

        mutable_fsm_state = state.get_mutable_abstract_state(self._fsm_state_idx)
        fsm_state_value: OpenLoopPlanarPushingPlannerState = context.get_abstract_state(
            self._fsm_state_idx
        ).get_value()

        if fsm_state_value == OpenLoopPlanarPushingPlannerState.PLAN_MOVE_TO_START:
            logging.info("Current state: PLAN_MOVE_TO_START")
            q_traj = self._plan_move_to_start(context)
            state.get_mutable_abstract_state(self._current_joint_traj_idx).set_value(
                TrajectoryWithTimingInformation(
                    trajectory=q_traj,
                    start_time_s=timing_information.start_move_to_start,
                )
            )
            timing_information.end_move_to_start = (
                timing_information.start_move_to_start + q_traj.end_time()
            )
            state.get_mutable_abstract_state(self._timing_information_idx).set_value(
                timing_information
            )

            logging.info("Transitioning to MOVE_TO_START FSM state.")
            mutable_fsm_state.set_value(OpenLoopPlanarPushingPlannerState.MOVE_TO_START)

        elif fsm_state_value == OpenLoopPlanarPushingPlannerState.MOVE_TO_START:
            # Add portion of '_wait_push_delay_s' as a margin
            if current_time <= (
                timing_information.end_move_to_start + 0.1 * self._wait_push_delay_s
            ):
                return

            logging.info("Transitioning to PLAN_PUSH FSM state.")
            mutable_fsm_state.set_value(OpenLoopPlanarPushingPlannerState.PLAN_PUSH)

        elif fsm_state_value == OpenLoopPlanarPushingPlannerState.PLAN_PUSH:
            timing_information.start_pushing = current_time + self._wait_push_delay_s
            timing_information.end_pushing = (
                timing_information.start_pushing
                + self._pushing_pose_trajectory.end_time()
            )
            state.get_mutable_abstract_state(self._timing_information_idx).set_value(
                timing_information
            )

            state.get_mutable_abstract_state(
                self._current_pose_trajectory_idx
            ).set_value(
                PiecewisePoseWithTimingInformation(
                    trajectory=self._pushing_pose_trajectory,
                    start_time_s=timing_information.start_pushing,
                )
            )

            logging.info("Transitioning to PUSH FSM state.")
            mutable_fsm_state.set_value(OpenLoopPlanarPushingPlannerState.PUSH)

        elif fsm_state_value == OpenLoopPlanarPushingPlannerState.PUSH:
            if current_time <= timing_information.end_pushing:
                return

            logging.info("Transitioning to FINISHED FSM state.")
            mutable_fsm_state.set_value(OpenLoopPlanarPushingPlannerState.FINISHED)

        elif fsm_state_value == OpenLoopPlanarPushingPlannerState.FINISHED:
            self._is_finished = True

        else:
            logging.error(f"Invalid FSM state: {fsm_state_value}")
            exit(1)

    def is_finished(self) -> bool:
        """Returns True if the task has been completed and False otherwise."""
        return self._is_finished


class OpenLoopPlanarPushingController(Diagram):
    """
    A controller that moves the robot to a given `pushing_start_pose` using a
    joint-space trajectory (NOTE that the trajectory is computed without collision
    checks). It then executes the provided `pushing_pose_trajectory` with a
    differential inverse kinematics controller.

    NOTE: The simulator context must be set using `set_context` before starting the
    simulation.
    """

    def __init__(
        self,
        controller_plant: MultibodyPlant,
        num_joint_positions: int,
        pushing_start_pose: RigidTransform,
        pushing_pose_trajectory: PiecewisePose,
        gripper_frame_name: str,
        initial_delay_s: float,
        wait_push_delay_s: float,
        move_to_start_velocity_limits: np.ndarray,
        move_to_start_acceleration_limits: np.ndarray,
    ):
        """
        Args:
            controller_plant (MultibodyPlant): The robot controller plant for
            controlling the robot.
            num_joint_positions (int): The number of robot joint positions.
            pushing_start_pose (RigidTransform): The pose that the robot should move to
            before executing the pushing trajectory.
            pushing_pose_trajectory (PiecewisePose): The pushing trajectory that will
            be executed using a differential inverse kinematics controller.
            gripper_frame_name (str): The name of the gripper frame that both the
            `pushing_start_pose` and the `pushing_pose_trajectory` poses refer to.
            initial_delay_s (float): The initial delay before starting a control action.
            wait_push_delay_s (float): The delay between moving to the
            `pushing_start_pose` and executing the `pushing_pose_trajectory`.
            move_to_start_velocity_limits (np.ndarray): The robot velocity limits
            for moving to the starting position. Shape (N,num_joint_positions) where N
            are the number of robot joints.
            move_to_start_acceleration_limits (np.ndarray): The robot acceleration
            limits for moving to the starting position. Shape (N,num_joint_positions)
            where N are the number of robot joints.
        """
        super().__init__()

        builder = DiagramBuilder()

        self._planer: OpenLoopPlanarPushingPlanner = builder.AddNamedSystem(
            "planer",
            OpenLoopPlanarPushingPlanner(
                num_joint_positions=num_joint_positions,
                controller_plant=controller_plant,
                pushing_start_pose=pushing_start_pose,
                pushing_pose_trajectory=pushing_pose_trajectory,
                gripper_frame_name=gripper_frame_name,
                initial_delay_s=initial_delay_s,
                wait_push_delay_s=wait_push_delay_s,
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

        diff_ik_pose_path_follower: DiffIKPathFollowingController = (
            builder.AddNamedSystem(
                "DiffIKPathFollowingController",
                DiffIKPathFollowingController(
                    controller_plant=controller_plant,
                    control_frame_name=gripper_frame_name,
                ),
            )
        )
        builder.Connect(
            self._planer.GetOutputPort("pose_trajectory"),
            diff_ik_pose_path_follower.GetInputPort("pose_trajectory"),
        )

        self._diff_ik_reseter: DiffIKReseter = builder.AddNamedSystem(
            "DiffIKReseter",
            DiffIKReseter(
                num_joint_positions=num_joint_positions,
                diff_ik_pose_path_follower=diff_ik_pose_path_follower,
            ),
        )
        builder.Connect(
            self._planer.GetOutputPort("reset_diff_ik"),
            self._diff_ik_reseter.GetInputPort("reset_diff_ik"),
        )
        builder.ConnectToSame(
            self._planer.GetInputPort("iiwa.position_measured"),
            self._diff_ik_reseter.GetInputPort("iiwa.position_measured"),
        )

        # Switch for switching between joint trajectory source and differential IK
        iiwa_position_command_switch: PortSwitch = builder.AddNamedSystem(
            "iiwa_position_command_switch", PortSwitch(num_joint_positions)
        )
        builder.Connect(
            self._planer.GetOutputPort("control_mode"),
            iiwa_position_command_switch.get_port_selector_input_port(),
        )
        builder.Connect(
            joint_traj_source.get_output_port(),
            iiwa_position_command_switch.DeclareInputPort("joint_traj_source"),
        )
        builder.Connect(
            diff_ik_pose_path_follower.get_output_port(),
            iiwa_position_command_switch.DeclareInputPort("differential_ik"),
        )
        builder.ExportOutput(
            iiwa_position_command_switch.get_output_port(),
            "iiwa.position",
        )

        builder.BuildInto(self)

    def is_finished(self) -> bool:
        """Returns True if the task has been completed and False otherwise."""
        return self._planer.is_finished()

    def set_context(self, context: Context) -> None:
        """
        Sets the simulator context. NOTE: This must be called before starting the
        simulation.
        """
        self._diff_ik_reseter.set_context(context)
