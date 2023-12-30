# iiwa_setup
iiwa real-world setup

## Installation

This repo uses Poetry for dependency management. To set up this project, first install
[Poetry](https://python-poetry.org/docs/#installation) and, make sure to have Python3.10
installed on your system.

Then, configure poetry to set up a virtual environment that uses Python 3.10:
```
poetry env use python3.10
```

Next, install all the required dependencies to the virtual environment with the
following command:
```bash
poetry install -vvv
```
(the `-vvv` flag adds verbose output).

For local Drake and manipulation installations, insert the following at the end of the
`.venv/bin/activate` and `.venv/bin/activate.nu` files, modifying the paths and python
version as required:
```bash
export PYTHONPATH=""
export PYTHONPATH=~/drake-build/install/lib/python3.10/site-packages:${PYTHONPATH}
export PYTHONPATH=~/manipulation:${PYTHONPATH}
```

Activate the environment:
```bash
poetry shell
```

Install `git lfs`:

```bash
git lfs install
git lfs pull
```

### iiwa Driver

[Drake's iiwa driver](https://github.com/RobotLocomotion/drake-iiwa-driver) must be
installed manually to use the real iiwa robot. NOTE that
[Drake's pre-requisites](https://drake.mit.edu/from_source.html) must be installed
before installing the driver.

The FRI source can be downloaded from
[here](https://mitprod-my.sharepoint.com/:u:/g/personal/nepfaff_mit_edu/EdUdfStUexZKqlfwKLTKOyUBmpoI3H1ylzit-813TMV1Eg?e=HRWaIv)
and installed using the following instructions:
```bash
cd kuka-fri
unzip /path/to/your/copy/of/FRI-Client-SDK_Cpp-1_7.zip
patch -p1 < ../fri_udp_connection_file_descriptor.diff
```

Once build, the driver can be run using `./bazel-bin/kuka-driver/kuka_driver` or using
`bazel run //kuka-driver:kuka_driver`.

#### Networking troubleshooting

If the driver doesn't connect to the kuka, check that the sunrise cabinet is reachable
on the network using `nmap -sP 192.170.10.2/24`. Both the local computer and a second
computer (the sunrise cabinet) should show up.

If it doesn't show up, check the following:
1. There must be an ethernet network connecting the local computer and the sunrise
sunrise cabinet KONI port (ideally through a switch). This network must have the static
IP `192.170.10.200` with netmask `255.255.255.0`.
2. The sunrise cabinet KONI port must be owned by RTOS and not by Windows. Connect a
monitor, mouse, and keyboard to the sunrise cabinet. Start the cabinet and login. Press
`WIN+R` to open the command window. Type
`C:\KUKA\Hardware\Manager\KUKAHardwareManager.exe -query OptionNIC -os RTOS`. Everything
is in order if the popup says `BusType OptionNIC found`. If the popup says
`BusTypeOptionNIC not present`, change the port ownership using
`C:\KUKA\Hardware\Manager\KUKAHardwareManager.exe -assign OptionNIC -os RTOS`. Unplug
the monitor and restart the sunrise cabinet before re-checking the network with `nmap`.

### Schunk WSG 50 Gripper Driver (Optional)

Connect the WSG gripper to the same switch that is connecting the local computer with
the sunrise cabinet. Add the IP `192.168.1.200` with netmask `255.255.255.0` as an
additional static IP to the network (the first IP should still be `192.170.10.200`).
The WSG is connected properly if the `WSG 50 Control Panel` web interface can be
accessed through http://192.168.1.20/.

[Drake's Schunk driver](https://github.com/RobotLocomotion/drake-schunk-driver) must be
installed manually to use the WSG programatically. Once build, the driver can be run
using `bazel run //src:schunk_driver`.

### Optitrack Driver (Optional)

[Drake's Optitrack driver](https://github.com/RobotLocomotion/optitrack-driver) must be
installed manually to use the [Optitrack](https://optitrack.com/) functionality.

Build and install the wheel as described
[here](https://github.com/RobotLocomotion/optitrack-driver#to-build-a-wheel). Make sure
to install the wheel from inside the poetry virtual environment.

## Executing code on the real robot

1. Start the torque or position driver on the teach pendant.
2. Run the iiwa driver by running `bazel run //kuka-driver:kuka_driver` from
`drake-iiwa-driver`.
3. If using the WSG, run the schunk driver using `bazel run //src:schunk_driver` from
`drake-schunk-driver`.
4. Run the desired script with the `--use_hardware` flag.

## Optitrack

To use the Optitrack system, run the Optitrack client from the
[driver]((https://github.com/RobotLocomotion/optitrack-driver)) directory:
```
bazel run //src:optitrack_client
```

### Inspecting Optitrack LCM messages

Clone [drake](https://github.com/RobotLocomotion/drake) and run LCM Spy from inside the
drake directory:
```bash
bazel run lcmtypes:drake-lcm-spy
```

### Calibrating Optitrack bodies

In a usual setup, we want to use Optitrack to update our internal model of the world
(multibody plant). However, the body poses returned by Optitrack assume different world
and body frames than the ones from our plant. Hence, we need to find the transform
between the Optitrack body pose and the plant body pose.

`scripts/calibrate_optitrack_body.py` can be used for computing this transform using the
the following procedure:

#### 1. Setup

Modify the `scenario_str` to contain the body of interest (currently
`sugar_box.dmd.yaml`) and a second (reference) version of that body (currently
`sugar_box_reference.dmd.yaml`). Both should be identical apart from the model name and
contain the SDFormat file of the body/ object.

Modify `object_name` and `ref_object_name` to match the model names of the first and
second version of the body.

Modify `ref_object_initial_positions` to position the body **above** the Optitrack
workspace (the object should fall on the floor and not start on the floor).

Modify `optitrack_iiwa_id` and `optitrack_body_id` to match the body IDs of the
Optitrack system. These can be identified as described
[here](#inspecting-optitrack-lcm-messages).

Set both `R_OptitrackBody_SimBody_W` and `p_OptitrackBody_SimBody_W` to `[0, 0, 0]`.

#### 2. Determine the reference body's positions

1. Set `is_init = True` and run the script.
2. Wait a few seconds until the printed pose stays approximately static. Then
note down the printed z-position and terminate the script.
3. Modify the reference body directive file to include a weld from the world frame to
the body frame. The weld transform should include the printed z-position, no rotation,
and planar positions that position the object close to the iiwa base inside the
Optitrack workspace.

#### 3. Determine the Optitrack body to plant-body transform

1. Place the real object at the planar position that corresponds to the weld from step 2.
Taking the iiwa center as the world origin and using a ruler to measure positions and
ensuring axis-aligned rotations should be helpful strategies here.
2. Remove the collision geometries from the body SDFormat file.
3. Set `is_init = False` and run the script.
4. Note down the printed transform and terminate the script.
5. Subtract the printed transform from the reference object weld transform
(element-wise subtraction of the positions and Euler angles) and use the result as
`R_OptitrackBody_SimBody_W` and `p_OptitrackBody_SimBody_W`.
6. Run the script and check if both bodies align. If they do, then
`R_OptitrackBody_SimBody_W` and `p_OptitrackBody_SimBody_W` represent the desired
transform.

### Recording Optitrack object pose data

The script `scripts/record_optitrack_body_pose_data.py` can be used for recording
the pose of an object using optitrack.

Example usage:
```bash
python scripts/record_optitrack_body_pose_data.py --out_path sugar_box_logs \
--object_directive "package://iiwa_setup/sugar_box.dmd.yaml" --object_name sugar_box \
--optitrack_object_id 3 --object_initial_positions '[1, 0, 0, 0, 0.5, 0.5, 0.025]' \
--p_optitrackBody_plantBody_world '[-0.03427348, 0.01983565, -0.01967432]' \
--R_optitrackBody_plantBody_world '[0.013, -0.035, 1.372]' --save_html
```

Note that data is only saved if the script is excited gracefully using the
`Stop Simulation` button in Meshcat.

### Simulating Optitrack Measurements

The script `scripts/simulate_optitrack.py` can be used to simulate Optitrack frame
measurements. The script either loads Optitrack frames and their associated publish
times from disk (arbitrary number of objects) or opens a GUI for interactively moving
a single object.

Example usage (disk):
```bash
python scripts/simulate_optitrack.py --optitrack_frames_path frames.npy \
--optitrack_frame_times_path frame_times.npy
```

Example usage (GUI):
```bash
python scripts/simulate_optitrack.py --initial_object_position '[0.5, 0.0, 0.15]' \
--initial_object_quaternion '[1.0, 0.0, 0.0, 0.0]' --optitrack_object_ids '[4, 3]'
```
