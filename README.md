# iiwa_setup
iiwa real world setup

## Installation

This repo uses Poetry for dependency management. To setup this project, first install
[Poetry](https://python-poetry.org/docs/#installation) and, make sure to have Python3.10
installed on your system.

Then, configure poetry to setup a virtual environment that uses Python 3.10:
```
poetry env use python3.10
```

Next, install all the required dependencies to the virtual environment with the
following command:
```
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
```
poetry shell
```

### iiwa Driver

[Drake's iiwa driver](https://github.com/RobotLocomotion/drake-iiwa-driver) must be installed manually to use the real iiwa robot.

The FRI source can be downloaded from [here](https://mitprod-my.sharepoint.com/:u:/g/personal/nepfaff_mit_edu/EdUdfStUexZKqlfwKLTKOyUBmpoI3H1ylzit-813TMV1Eg?e=HRWaIv) and installed using the following instructions:
```bash
cd kuka-fri
unzip /path/to/your/copy/of/FRI-Client-SDK_Cpp-1_7.zip
patch -p1 < ../fri_udp_connection_file_descriptor.diff
```

### Optitrack (Optional)

[Drake's optitrack driver](https://github.com/RobotLocomotion/optitrack-driver) must be
installed manually to use the optitrack functionality.

Build and install the wheel as described
[here](https://github.com/RobotLocomotion/optitrack-driver#to-build-a-wheel). Make sure
to install the wheel from inside the poetry virtual environment.

## Executing code on the real robot

1. Start the torque or position driver on the teach pendant.
2. Run the iiwa driver by running `bazel run //kuka-driver:kuka_driver` from `drake-iiwa-driver`.
3. Run the desired script with the `--use_hardware` flag.
