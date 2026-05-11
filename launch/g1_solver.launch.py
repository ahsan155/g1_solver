import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("g1_solver")
    default_task_composer = os.path.join(pkg_share, "config", "task_composer_plugins.yaml")

    task_composer_arg = DeclareLaunchArgument(
        "task_composer_config",
        default_value=default_task_composer,
        description="Path to Tesseract task composer YAML (TESSERACT_TASK_COMPOSER_CONFIG_FILE).",
    )

    env_task_composer = SetEnvironmentVariable(
        name="TESSERACT_TASK_COMPOSER_CONFIG_FILE",
        value=LaunchConfiguration("task_composer_config"),
    )

    main_node = Node(
        package="g1_solver",
        executable="g1_solver_main",
        name="g1_solver",
        output="screen",
    )

    return LaunchDescription(
        [
            task_composer_arg,
            env_task_composer,
            main_node,
        ]
    )
