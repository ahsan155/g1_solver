# g1_solver

ROS 2 Python package that runs a [Tesseract](https://tesseract-robotics.github.io/) task-composer pipeline (default: OMPL) for the Unitree G1 left arm. It subscribes to `/joint_states`, plans Cartesian segments defined in `src/joint_table_target.py`, and publishes commands on `/arm_joint_cmd` as `std_msgs/Float32MultiArray`.

## Layout

| Path | Purpose |
|------|---------|
| `src/` | Python modules; entry node is `main.py` (installed as executable `g1_solver_main`) |
| `config/` | Solver YAML, task composer plugins, contact manager plugins |
| `description/` | URDF/SRDF and bundled mesh/description assets resolved via `package://g1_solver/...` |
| `launch/` | `g1_solver.launch.py` starts the main node and sets `TESSERACT_TASK_COMPOSER_CONFIG_FILE` |

## Dependencies

**ROS 2** (tested with Humble/Iron/Jazzy-style `ament_cmake` workflows):

- `rclpy`, `sensor_msgs`, `std_msgs`, `ament_index_python`, `launch`, `launch_ros`

**Python** (must be available in the same environment used for Tesseract):

- `numpy`, `pyyaml`
- `tesseract_robotics` bindings (`tesseract_environment`, `tesseract_task_composer`, `tesseract_command_language`, …)
- `tesseract_robotics_viewer` (optional for visualization; required by current `main.py`)

Install Tesseract Python packages from your workspace/build instructions; they are not always expressible as plain `rosdep` keys.

## Build

Put this folder in a ROS 2 workspace `src/` tree (or add it as a symlink), install dependencies, then build:

```bash
cd /path/to/your_ros2_ws
source /opt/ros/$ROS_DISTRO/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select g1_solver
source install/setup.bash
```

## Run

Launch file (recommended): sets `TESSERACT_TASK_COMPOSER_CONFIG_FILE` to `share/g1_solver/config/task_composer_plugins.yaml` by default.

```bash
ros2 launch g1_solver g1_solver.launch.py
```

Override the task composer YAML:

```bash
ros2 launch g1_solver g1_solver.launch.py task_composer_config:=/absolute/path/to/plugins.yaml
```

Run the node executable directly (you must set the environment variable yourself):

```bash
export TESSERACT_TASK_COMPOSER_CONFIG_FILE=$(ros2 pkg prefix g1_solver)/share/g1_solver/config/task_composer_plugins.yaml
ros2 run g1_solver g1_solver_main
```

### Mock `/joint_states`

For bench testing without hardware:

```bash
ros2 run g1_solver g1_solver_mock_joint_states
```

## Runtime expectations

- **`/joint_states`**: required before planning (see `joint_states_timeout_s` in `config/solver_config.yaml`).
- **`/arm_joint_cmd`**: `Float32MultiArray` with 17 joints in the order defined in `joint_table_target.py` (`UNITREE_CMD_JOINTS`).

Robot URDF/SRDF URLs are configured in `config/solver_config.yaml` using `package://g1_solver/description/...`.

## Maintainer / license

Replace `maintainer@example.com` / maintainer name in `package.xml` and `setup.py` before publishing. Default license in manifests is `Apache-2.0`; change if needed.

## Note on `test.py`

The file `test.py` at the package root is a standalone scratch script and is **not** installed by this package. Run it only if you adjust imports and `PYTHONPATH` yourself, or move it under `src/` as a proper module.
