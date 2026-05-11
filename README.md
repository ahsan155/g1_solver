# g1_solver

ROS 2 Python package that runs a [Tesseract](https://tesseract-robotics.github.io/) task-composer pipeline (default: OMPL) for the Unitree G1 left arm. It subscribes to `/joint_states`, plans Cartesian segments defined in `src/joint_table_target.py`, and publishes commands on `/arm_joint_cmd` as `std_msgs/Float32MultiArray`.

## Layout

| Path | Purpose |
|------|---------|
| `src/` | Python modules; entry node is `main.py` (installed as executable `g1_solver_main`) |
| `config/` | Solver YAML, task composer plugins, contact manager plugins |
| `description/` | URDF/SRDF and bundled mesh/description assets resolved via `package://g1_solver/...` |
| `launch/` | `g1_solver.launch.py` starts the main node and sets `TESSERACT_TASK_COMPOSER_CONFIG_FILE`; `mock_joint_states.launch.py` publishes mock `/joint_states` for testing |

## Dependencies

**ROS 2** (tested with Humble/Iron/Jazzy-style `ament_cmake` workflows):

- `rclpy`, `sensor_msgs`, `std_msgs`, `ament_index_python`, `launch`, `launch_ros`

**Python** (must be available in the same interpreter/environment you use to run the node):

On Ubuntu 22.04 / 24.04, install base tools and NumPy, then install the Tesseract Python wheels from PyPI:

```bash
sudo apt install python3-pip python3-numpy
python3 -m pip install -U pip
python3 -m pip install --user pyyaml
python3 -m pip install --user tesseract_robotics tesseract_robotics_viewer
```

Notes:

- `pyyaml` is the PyPI name for the `yaml` module (`import yaml`).
- `tesseract_robotics_viewer` is required by the current `main.py` (viewer dependency).

## Build

Put this folder in a ROS 2 workspace `src/` tree (or add it as a symlink), install dependencies, then build:

```bash
cd /path/to/your_ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select g1_solver
source install/setup.bash
```

## Set environment variables

Tesseract resolves `package://g1_solver/...` URIs using `TESSERACT_RESOURCE_PATH`. Point it at the **parent directory of your ROS 2 workspace `src/`** (the folder that contains `g1_solver` as a subdirectory), or otherwise at a path where the `g1_solver` package folder is discoverable per your locator setup.

Example (adjust paths to match your machine):

```bash
export TESSERACT_RESOURCE_PATH=~/ros_ws/src
export TESSERACT_TASK_COMPOSER_CONFIG_FILE=~/ros_ws/src/g1_solver/config/task_composer_plugins_no_trajopt_ifopt.yaml
```

If you use the provided launch file for the main node, you typically **do not** need to set `TESSERACT_TASK_COMPOSER_CONFIG_FILE` manually (see below); keep `TESSERACT_RESOURCE_PATH` set whenever you use `package://...` URIs in `solver_config.yaml`.

## Run

Launch file (recommended): sets `TESSERACT_TASK_COMPOSER_CONFIG_FILE` to `share/g1_solver/config/task_composer_plugins.yaml` by default.

```bash
ros2 launch g1_solver g1_solver.launch.py
```

Override the task composer YAML:

```bash
ros2 launch g1_solver g1_solver.launch.py task_composer_config:=/absolute/path/to/plugins.yaml
```

### Mock `/joint_states`

For bench testing without hardware:

```bash
ros2 launch g1_solver mock_joint_states.launch.py
```

## Runtime expectations

- **`/joint_states`**: required before planning (see `joint_states_timeout_s` in `config/solver_config.yaml`).
- **`/arm_joint_cmd`**: `Float32MultiArray` with 17 joints in the order defined in `joint_table_target.py` (`UNITREE_CMD_JOINTS`).

Robot URDF/SRDF URLs are configured in `config/solver_config.yaml` using `package://g1_solver/description/...`.
