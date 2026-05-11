import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from .joint_table_target import UNITREE_CMD_JOINTS
from .joint_table_target import UNITREE_LIMITS

ARM_CMD_TOPIC = "/arm_joint_cmd"
JOINT_STATES_TOPIC = "/joint_states"


class ArmJointCmdBridge(Node):
    def __init__(self):
        super().__init__("tesseract_arm_cmd_bridge")
        self.pub = self.create_publisher(Float32MultiArray, ARM_CMD_TOPIC, 10)
        self.sub = self.create_subscription(JointState, JOINT_STATES_TOPIC, self._joint_states_cb, 10)
        self.latest_joint_map = {}
        self.cmd17 = np.zeros(17, dtype=np.float64)
        self.received_feedback = False
        self.cmd_name_to_idx = {name: i for i, name in enumerate(UNITREE_CMD_JOINTS)}

    def _joint_states_cb(self, msg: JointState):
        self.received_feedback = True
        for n, p in zip(msg.name, msg.position):
            self.latest_joint_map[n] = float(p)

        # Keep non-commanded joints from latest measured robot state.
        for i, joint_name in enumerate(UNITREE_CMD_JOINTS):
            if joint_name in self.latest_joint_map:
                self.cmd17[i] = self.latest_joint_map[joint_name]

    def wait_for_feedback(self, timeout_s: float = 3.0):
        start = time.monotonic()
        while not self.received_feedback and (time.monotonic() - start) < timeout_s:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.received_feedback

    def clamp_value(self, index: int, value: float) -> float:
        lo, hi = UNITREE_LIMITS[index]
        clamped = min(max(value, lo), hi)
        if clamped != value:
            self.get_logger().warn(
                f"Clamped {UNITREE_CMD_JOINTS[index]} from {value:.4f} to {clamped:.4f} within [{lo:.4f}, {hi:.4f}]"
            )
        return clamped

    def publish_left_arm_state(self, left_arm_names, left_arm_values):
        # Refresh with latest feedback before overlaying left arm command.
        rclpy.spin_once(self, timeout_sec=0.0)
        for jn, jv in zip(left_arm_names, left_arm_values):
            if jn not in self.cmd_name_to_idx:
                continue
            idx = self.cmd_name_to_idx[jn]
            self.cmd17[idx] = self.clamp_value(idx, float(jv))

        msg = Float32MultiArray()
        msg.data = [float(v) for v in self.cmd17]
        self.pub.publish(msg)

def left_arm_vector_from_joint_states(names, joint_map):
    q = np.zeros(len(names), dtype=np.float64)
    missing = []
    for i, name in enumerate(names):
        if name in joint_map:
            q[i] = joint_map[name]
        else:
            missing.append(name)
    if missing:
        print(f"Warning: missing joint positions in /joint_states for FK seed: {missing}")
    return q