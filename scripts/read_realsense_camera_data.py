"""An example script for reading intel realsense camera data.
NOTE: This currently only works in simulation
"""

import argparse

from typing import Tuple

import numpy as np

from manipulation.station import LoadScenario
from matplotlib import pyplot as plt
from pydrake.all import (
    ApplySimulatorConfig,
    Context,
    DiagramBuilder,
    PointCloud,
    Simulator,
)

from iiwa_setup.iiwa import IiwaHardwareStationDiagram


def get_image_data(
    station: IiwaHardwareStationDiagram, context: Context
) -> Tuple[np.ndarray, np.ndarray]:
    color_image = station.GetOutputPort("camera0.rgb_image").Eval(context)
    depth_image = station.GetOutputPort("camera0.depth_image").Eval(context)
    return color_image.data.copy(), depth_image.data.copy()


def main(scenario_str: str, use_hardware: bool) -> None:
    builder = DiagramBuilder()

    scenario = LoadScenario(data=scenario_str)
    station: IiwaHardwareStationDiagram = builder.AddNamedSystem(
        "station",
        IiwaHardwareStationDiagram(
            scenario=scenario,
            has_wsg=False,
            use_hardware=use_hardware,
            create_point_clouds=True,
        ),
    )

    diagram = builder.Build()

    simulator = Simulator(diagram)
    ApplySimulatorConfig(scenario.simulator_config, simulator)
    context = simulator.get_context()

    # Initialize image plots
    station_context = station.GetMyContextFromRoot(context)
    station.ForcedPublish(station_context)
    color_img_data, depth_img_data = get_image_data(station, station_context)

    ax1 = plt.subplot(1, 2, 1)
    ax2 = plt.subplot(1, 2, 2)
    color_image_plot = ax1.imshow(color_img_data)
    depth_image_plot = ax2.imshow(depth_img_data)
    plt.ion()  # Interactive mode to allow updating the plots

    station.internal_meshcat.AddButton("Stop Simulation")
    while station.internal_meshcat.GetButtonClicks("Stop Simulation") < 1:
        simulator.AdvanceTo(context.get_time() + 0.1)
        color_img_data, depth_img_data = get_image_data(station, station_context)
        color_image_plot.set_data(color_img_data)
        depth_image_plot.set_data(depth_img_data)
        plt.pause(0.1)
    station.internal_meshcat.DeleteButton("Stop Simulation")

    plt.ioff()
    # NOTE: This will block the script until the plots are closed
    plt.show()

    # Demonstrate reading point cloud data
    # NOTE: Point clouds are also visualized in the external station meshcat
    pcd: PointCloud = station.GetOutputPort("camera0.point_cloud").Eval(station_context)
    pcd_point_coordinates = pcd.xyzs()
    # pcd_point_colors = pcd.rgbs()
    print(f"Number of points in the point cloud: {pcd_point_coordinates.shape[1]}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--use_hardware",
        action="store_true",
        help="Whether to use real world hardware.",
    )
    args = parser.parse_args()

    scenario_str = f"""
    directives:
    - add_directives:
        file: package://iiwa_setup/iiwa7_with_planar_pusher.dmd.yaml
    - add_model:
        name: camera
        file: package://manipulation/camera_box.sdf
    - add_weld:
        parent: world
        child: camera::base
        X_PC:
            translation: [1.5, 0.1, 0.5]
            rotation: !Rpy {{deg: [-90, 0, 100]}}
    plant_config:
        time_step: 1e-5
        contact_model: "hydroelastic"
        discrete_contact_approximation: "sap"
    cameras:
        camera0:
            name: camera0
            depth: True
            X_PB:
                base_frame: camera::base
    model_drivers:
        iiwa: !IiwaDriver
            lcm_bus: "default"
            control_mode: position_only
    lcm_buses:
        default:
            lcm_url: ""
    """
    main(scenario_str=scenario_str, use_hardware=args.use_hardware)
