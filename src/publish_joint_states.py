import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from .joint_table_target import ALL_JOINT_STATES_JOINTS


class JointStatesPublisher(Node):
    def __init__(self):
        super().__init__("mock_joint_states_publisher")
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.joint_names = list(ALL_JOINT_STATES_JOINTS)
        self.timer = self.create_timer(1.0 / 20.0, self.publish_joint_states)
        self.get_logger().info(
            f"Publishing /joint_states with {len(self.joint_names)} joints at 20 Hz"
        )

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names

        # Small near-zero values for mock state.
        n = len(self.joint_names)
        msg.position = [0.01] * n
        msg.velocity = [0.0] * n
        msg.effort = [0.0] * n

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JointStatesPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
