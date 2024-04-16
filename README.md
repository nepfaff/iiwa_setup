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

Install `git-lfs`:

```bash
git-lfs install
git-lfs pull
```

### iiwa Driver

[Drake's iiwa driver](https://github.com/RobotLocomotion/drake-iiwa-driver) must be
installed manually to use the real iiwa robot. NOTE that
[Drake's pre-requisites](https://drake.mit.edu/from_source.html) must be installed
before installing the driver.

The FRI source can be downloaded from
[here](https://mitprod-my.sharepoint.com/:u:/g/personal/nepfaff_mit_edu/EdUdfStUexZKqlfwKLTKOyUBmpoI3H1ylzit-813TMV1Eg?e=HRWaIv)
and installed using the following instructions (from the driver repo):
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

#### Port troubleshooting

Make sure that the sunrise cabinet port matches the kuka driver port. If not, then
modify the kuka driver source code to change the port
(`kuka-driver/kuka_driver.cc/kDefaultPort`). It is also possible to start the kuka
driver with a specific port over the command line. However, it is easier to hardcode the
port as the cabinet port won't change.

#### Robot limit exceeded errors

1. Enter the KRF mode
2. Manually operate the robot out of the limits using the tablet
3. Re-enter automatic mode

If no KRF mode exists, then do the following:
1. Unmaster the joint whose limits are exceeded
2. Use the teach pendant in T1 mode to move the joint back inside its allowed range
3. Master the joint

#### "Voltage of intermediate circuit too low" error

One of the fuses is blown. You need to open the control box and replace them.

The two fuses are:
- 5A/80V Automotive Fuse ([buying link](https://www.newark.com/multicomp-pro/mp008147/fuse-automotive-5a-80vdc-rohs/dp/69AJ0523))
- 7.5A/80V Automotive Fuse ([buying link](https://www.newark.com/multicomp-pro/mp008148/fuse-automotive-7-5a-80vdc-rohs/dp/69AJ0524))

### Schunk WSG 50 Gripper Driver (Optional)

Connect the WSG gripper to the same switch that is connecting the local computer with
the sunrise cabinet. Add the IP `192.168.1.200` with netmask `255.255.255.0` as an
additional static IP to the network (the first IP should still be `192.170.10.200`).
The WSG is connected properly if the `WSG 50 Control Panel` web interface can be
accessed through http://192.168.1.20/. Try to control the gripper through the web
interface. If this doesn't work, then controlling it through the driver also won't work.

[Drake's Schunk driver](https://github.com/RobotLocomotion/drake-schunk-driver) must be
installed manually to use the WSG programatically. Once built, the driver can be run
using `bazel run //src:schunk_driver`. The driver requires Bazel 6. Multiple Bazel
versions can be managed by installing `bazelisk from [here](https://github.com/bazelbuild/bazelisk/releases).
The Bazel version will then be read from the `.bazeliskrc` file in the repo.

#### Networking troubeshooting

Check that one host shows up when using `nmap -sP 192.168.1.201`. and that the website
is accessible at http://192.168.1.20/. If not, then check that you followed the IP
instructions and that the gripper's ethernet cable is plugged into the switch.

#### Error while moving: The device is not initialized

1. Navigate to the website http://192.168.1.20/
2. Motion -> Manual Control
3. Click on `Home` and wait until the homing sequence is finished
4. Re-try commanding the gripper via the web interface

#### Getting system info failed

Follow the [wsg driver setup instructions](https://github.com/RobotLocomotion/drake-schunk-driver?tab=readme-ov-file#configuring-the-gripper).
In particular, the gripper might be set to ICP instead of UDP.

### Optitrack Driver (Optional)

[Drake's Optitrack driver](https://github.com/RobotLocomotion/optitrack-driver) must be
installed manually to use the [Optitrack](https://optitrack.com/) functionality.

Build and install the wheel as described
[here](https://github.com/RobotLocomotion/optitrack-driver#to-build-a-wheel). Make sure
to install the wheel from inside the poetry virtual environment.

### FT 300-S Driver (Optional)

The [FT 300-S LCM driver](https://github.com/nepfaff/ft-300s-driver) must be installed
according to its README instructions.

The included LCM messages must be added to the python path (after building):
```
export PYTHONPATH=~/path_to_parent_dir/ft_300s_driver/bazel-bin/lcmtypes/ft_300s/:${PYTHONPATH}
```

## Executing code on the real robot

1. Start the `DrakeFRIPositionDriver` or `DrakeDRITorqueDriver` on the teach pendant.
2. Run the iiwa driver by running `bazel run //kuka-driver:kuka_driver` from
`drake-iiwa-driver`.
3. If using the WSG, run the schunk driver using `bazel run //src:schunk_driver` from
`drake-schunk-driver`.
4. Run the desired script with the `--use_hardware` flag.

### Controlling the robot in `torque_only` mode

**NOTE:** It is recommended to calibrate the joint torque sensors before running the
robot in `torque_only` mode. This can be achieved by running the
`PositionAndGMSReferencing` application on the teach pendant.

1. Start the `DrakeFRITorqueOnlyDriver` on the teach pendant.
2. Optional: Make sure that the iiwa driver is build by running `bazel build //...` from
`drake-iiwa-driver`.
3. Run the iiwa driver by running
`sudo ./bazel-bin/kuka-driver/kuka_driver --torque_only=true --time_step 0.001 --realtime`
from `drake-iiwa-driver` (`sudo` is required for `--realtime` which helps but is not
required).
4. Run the desired script with the `--use_hardware` flag.

#### Obtaining slightly better performance

You might be able to achieve slightly better performance in `torque_only` mode by
pinning the processes to the same core and increasing their priority.

1. Make sure that you can run `chrt` without sudo privileges:
`sudo setcap cap_sys_nice=eip /usr/bin/chrt`. This is only required once.
2. Run the desired script using `taskset -c 1,29 chrt -r 90 python {...} --use_hardware`.

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

Make sure that the optitrack iiwa frame is in the middle of the iiwa (in the middle of
the 4 corner points of which 3 are optitrack markers). By default, the frame will be in
the middle of the 3 optitrack markers, which is slightly different. It can be changed
using the optitrack GUI.

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

Make sure that `X_oB_pB = RigidTransform([0, 0, 0])` is uncommented.

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
5. Set `X_W_oB` to the printed transform.
6. Comment out `X_oB_pB = RigidTransform([0, 0, 0])`.
7. Run the script and check if both bodies align. If they do, then `X_oB_pB` represents
the desired transform.

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
