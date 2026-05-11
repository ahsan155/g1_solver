from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Publish mock /joint_states for testing g1_solver without hardware."""
    return LaunchDescription(
        [
            Node(
                package="g1_solver",
                executable="g1_solver_mock_joint_states",
                name="mock_joint_states_publisher",
                output="screen",
            ),
        ]
    )
