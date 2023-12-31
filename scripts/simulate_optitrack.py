"""Script for simulating Optitrack measurements."""

import argparse
import json
import logging

from typing import List, Optional

import numpy as np

from optitrack import optitrack_frame_t
from pydrake.all import (
    AbstractValue,
    ConstantValueSource,
    DiagramBuilder,
    DrakeLcm,
    LcmInterfaceSystem,
    LcmPublisherSystem,
    MeshcatPoseSliders,
    Quaternion,
    RigidTransform,
    Simulator,
    StartMeshcat,
)

from iiwa_setup.sensors import OptitrackFrameSource, PosesToOptitrackFrameConverter
from iiwa_setup.util import AbstractMultiplexer


def main(
    lcm_publish_period: float,
    optitrack_frames: Optional[List[optitrack_frame_t]] = None,
    optitrack_frame_times: Optional[List[float]] = None,
    iiwa_pose: Optional[RigidTransform] = None,
    initial_object_pose: Optional[RigidTransform] = None,
    optitrack_object_ids: Optional[List[int]] = None,
):
    """Simulate optitrack measurements.
    If `optitrack_frames` and `optitrack_frame_times` are not None, then these are
    frames are published. Otherwise, the optitrack measurements are set using the
    meshcat GUI (requires `iiwa_pose`, `initial_object_pose`, and `optitrack_object_ids`
    to be set).

    Args:
        lcm_publish_period (float): The optitrack frame publish period.
        optitrack_frames (Optional[List[optitrack_frame_t]]): The optitrack frames to
            publish.
        optitrack_frame_times (Optional[List[float]]): The times at which to publish
            the optitrack frames.
        iiwa_pose (Optional[RigidTransform]): The iiwa pose.
        initial_object_pose (Optional[RigidTransform]): The initial object pose.
        optitrack_object_ids (Optional[List[int]]): The optitrack object IDs.
    """
    builder = DiagramBuilder()

    lcm = DrakeLcm()
    lcm_system = builder.AddSystem(LcmInterfaceSystem(lcm))

    optitrack_frame_publisher: LcmPublisherSystem = builder.AddNamedSystem(
        "OptitrackFramePublisher",
        LcmPublisherSystem.Make(
            channel="OPTITRACK_FRAMES",
            lcm_type=optitrack_frame_t,
            lcm=lcm_system,
            use_cpp_serializer=False,
            publish_period=lcm_publish_period,
        ),
    )

    if optitrack_frames is not None and optitrack_frame_times is not None:
        logging.info("Using pre-recorded Optitrack frames.")

        optitrack_frame_source: OptitrackFrameSource = builder.AddNamedSystem(
            "OptitrackFrameSource",
            OptitrackFrameSource(
                optitrack_frames=optitrack_frames,
                optitrack_frame_times=optitrack_frame_times,
            ),
        )
        builder.Connect(
            optitrack_frame_source.get_output_port(),
            optitrack_frame_publisher.get_input_port(),
        )
    else:
        logging.info("Using meshcat GUI Optitrack frames.")

        assert (
            iiwa_pose is not None
            and initial_object_pose is not None
            and optitrack_object_ids is not None
        ), (
            "Must set iiwa_pose, initial_object_pose, and optitrack_object_ids when "
            + "using GUI!"
        )

        # Set up pose sources
        iiwa_pose_source: ConstantValueSource = builder.AddSystem(
            ConstantValueSource(AbstractValue.Make(iiwa_pose))
        )
        meshcat = StartMeshcat()
        # TODO: Allow multiple objects using the MeshcatPoseSliders' 'prefix' argument
        object_pose_slider: MeshcatPoseSliders = builder.AddSystem(
            MeshcatPoseSliders(meshcat=meshcat, initial_pose=initial_object_pose)
        )

        # Concatenate iiwa and object poses
        mux: AbstractMultiplexer = builder.AddSystem(
            AbstractMultiplexer(
                num_abstract_scalar_inputs=2, abstract_type=RigidTransform
            )
        )
        builder.Connect(
            iiwa_pose_source.get_output_port(),
            mux.get_input_port(0),
        )
        builder.Connect(
            object_pose_slider.get_output_port(),
            mux.get_input_port(1),
        )

        # Convert poses to optitrack frame
        poses_to_optitrack_frame_converter: PosesToOptitrackFrameConverter = (
            builder.AddSystem(
                PosesToOptitrackFrameConverter(
                    optitrack_object_ids=optitrack_object_ids
                )
            )
        )
        builder.Connect(
            mux.get_output_port(),
            poses_to_optitrack_frame_converter.get_input_port(),
        )
        builder.Connect(
            poses_to_optitrack_frame_converter.get_output_port(),
            optitrack_frame_publisher.get_input_port(),
        )

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)
    context = simulator.get_context()
    logging.info("Entering infinite publish loop. Terminate script to exit.")
    while True:
        simulator.AdvanceTo(context.get_time() + lcm_publish_period)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--lcm_publish_period",
        type=float,
        default=0.1,
        help="The optitrack frame publish period.",
    )
    parser.add_argument(
        "--optitrack_frames_path",
        type=str,
        default=None,
        help="The path to the optitrack frames to publish. Must be saved using `np.save`.",
    )
    parser.add_argument(
        "--optitrack_frame_times_path",
        type=str,
        default=None,
        help="The path to the optitrack frame times to publish. Indices must "
        + "correspond to the ones in `optitrack_frames_path`. Must be saved using "
        + "`np.save`.",
    )
    parser.add_argument(
        "--iiwa_position",
        type=json.loads,
        default=[0.0, 0.0, 0.0],
        help="The static iiwa position of form [x, y, z]. Only used if "
        + "`optitrack_frames_path` is None.",
    )
    parser.add_argument(
        "--iiwa_quaternion",
        type=json.loads,
        default=[1.0, 0.0, 0.0, 0.0],
        help="The static iiwa rotation of form [qw, qx, qy, qz]. Only used if "
        + "`optitrack_frames_path` is None.",
    )
    parser.add_argument(
        "--initial_object_position",
        type=json.loads,
        default=None,
        help="The initial object position of form [x, y, z]. Only used if "
        + "`optitrack_frames_path` is None.",
    )
    parser.add_argument(
        "--initial_object_quaternion",
        type=json.loads,
        default=None,
        help="The initial object rotation of form [qw, qx, qy, qz]. Only used if "
        + "`optitrack_frames_path` is None.",
    )
    parser.add_argument(
        "--optitrack_object_ids",
        type=json.loads,
        default=None,
        help="The optitrack object IDs of form [iiwa_id, object_id]. Only used if "
        + "`optitrack_frames_path` is None.",
    )
    parser.add_argument(
        "--log_level",
        type=str,
        default="INFO",
        choices=["CRITICAL", "ERROR", "WARNING", "INFO", "DEBUG"],
        help="Log level.",
    )
    args = parser.parse_args()

    logging.basicConfig(level=args.log_level)

    if args.optitrack_frames_path is not None:
        assert (
            args.optitrack_frame_times_path is not None
        ), "Must set frame times when using saved optitrack frames!"
        iiwa_pose, initial_object_pose = None, None
        optitrack_frames = np.load(args.optitrack_frames_path, allow_pickle=True)
        optitrack_frame_times = np.load(
            args.optitrack_frame_times_path, allow_pickle=True
        )
    else:
        optitrack_frames, optitrack_frame_times = None, None
        iiwa_pose = RigidTransform(Quaternion(args.iiwa_quaternion), args.iiwa_position)
        initial_object_pose = RigidTransform(
            Quaternion(args.initial_object_quaternion), args.initial_object_position
        )

    main(
        lcm_publish_period=args.lcm_publish_period,
        optitrack_frames=optitrack_frames,
        optitrack_frame_times=optitrack_frame_times,
        iiwa_pose=iiwa_pose,
        initial_object_pose=initial_object_pose,
        optitrack_object_ids=args.optitrack_object_ids,
    )
