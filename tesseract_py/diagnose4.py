# created ROS node to publish left arm joint commands while subscribing to /joint_states


import re
import traceback
import os
import time
import numpy as np
import numpy.testing as nptest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_environment import Environment, AnyPoly_wrap_EnvironmentConst
from tesseract_robotics.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond, \
    ManipulatorInfo, AnyPoly, AnyPoly_wrap_double
from tesseract_robotics.tesseract_kinematics import KinGroupIKInput, KinGroupIKInputs
from tesseract_robotics.tesseract_command_language import CartesianWaypoint, WaypointPoly, \
    MoveInstructionType_FREESPACE, MoveInstruction, InstructionPoly, StateWaypoint, StateWaypointPoly, \
    CompositeInstruction, MoveInstructionPoly, CartesianWaypointPoly, ProfileDictionary, \
        AnyPoly_as_CompositeInstruction, CompositeInstructionOrder_ORDERED, DEFAULT_PROFILE_KEY, \
        AnyPoly_wrap_CompositeInstruction, DEFAULT_PROFILE_KEY, JointWaypoint, JointWaypointPoly, \
        InstructionPoly_as_MoveInstructionPoly, WaypointPoly_as_StateWaypointPoly, \
        MoveInstructionPoly_wrap_MoveInstruction, StateWaypointPoly_wrap_StateWaypoint, \
        CartesianWaypointPoly_wrap_CartesianWaypoint, JointWaypointPoly_wrap_JointWaypoint, \
        AnyPoly_wrap_ProfileDictionary

from tesseract_robotics.tesseract_task_composer import TaskComposerPluginFactory, \
    TaskComposerDataStorage, TaskComposerContext

from tesseract_robotics_viewer import TesseractViewer
from tesseract_robotics import tesseract_common
from tesseract_robotics.tesseract_collision import ContactResultMap, ContactRequest, ContactResultVector, ContactTestType_ALL


#tesseract_common.setLogLevel(tesseract_common.CONSOLE_BRIDGE_LOG_DEBUG)

OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask"
TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"
ARM_CMD_TOPIC = "/arm_joint_cmd"
JOINT_STATES_TOPIC = "/joint_states"

# Unitree /arm_joint_cmd expects exactly these 17 joints in this order.
UNITREE_CMD_JOINTS = [
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

# Index-aligned limits for /arm_joint_cmd values.
UNITREE_LIMITS = [
    (-2.5, 2.5),
    (-0.52, 0.52),
    (-0.52, 0.52),
    (-2.9, 2.5),
    (-1.4, 2.1),
    (-2.5, 2.5),
    (-0.9, 2.0),
    (-1.8, 1.8),
    (-1.614, 1.614),
    (-1.614, 1.614),
    (-2.9, 2.5),
    (-2.1, 1.4),
    (-2.5, 2.5),
    (-0.9, 2.0),
    (-1.8, 1.8),
    (-1.614, 1.614),
    (-1.614, 1.614),
]


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

task_composer_filename = os.environ["TESSERACT_TASK_COMPOSER_CONFIG_FILE"]
rclpy.init()
ros_bridge = ArmJointCmdBridge()

# Initialize the resource locator and environment
locator = GeneralResourceLocator()
abb_irb2400_urdf_package_url = "package://tesseract/mycode/resource/g1.urdf"
abb_irb2400_srdf_package_url = "package://tesseract/mycode/resource/g1.srdf"
abb_irb2400_urdf_fname = FilesystemPath(locator.locateResource(abb_irb2400_urdf_package_url).getFilePath())
abb_irb2400_srdf_fname = FilesystemPath(locator.locateResource(abb_irb2400_srdf_package_url).getFilePath())


t_env = Environment()

# locator_fn must be kept alive by maintaining a reference
assert t_env.init(abb_irb2400_urdf_fname, abb_irb2400_srdf_fname, locator)

# Fill in the manipulator information. This is used to find the kinematic chain for the manipulator. This must
# match the SRDF, although the exact tcp_frame can differ if a tool is used.
manip_info = ManipulatorInfo()
manip_info.tcp_frame = "left_wrist_yaw_link"
manip_info.manipulator = "left_arm"
manip_info.working_frame = "torso_link"

# Create a viewer and set the environment so the results can be displayed later
viewer = TesseractViewer()
viewer.update_environment(t_env, [0,0,0])


# Start the viewer
viewer.start_serve_background()

# Set the initial state of the robot in both viewer and environment
# initial state is 0 for all joints
joint_names = list(t_env.getActiveJointNames())
joint_positions = np.zeros(len(joint_names))
viewer.update_joint_positions(joint_names, joint_positions) 
t_env.setState(joint_names, joint_positions)


# getting cartesian pose from given joint positions of left_wrist_yaw_link
kg = t_env.getKinematicGroup("left_arm")
left_arm_names = list(t_env.getGroupJointNames("left_arm")) 
left_arm_pos = np.ones(len(left_arm_names)) * 0.0
fk = kg.calcFwdKin(left_arm_pos)
T = fk["left_wrist_yaw_link"]
R = T.rotation()
print('left_wrist_yaw_link matrix',T.matrix())

# Quick IK sweep on y to estimate reachable boundary for current orientation.
base_x = 0.199774285
base_y = 0.148661704
base_z = 0.0952328317

#boundaries(robots perspective) from all 0 joint position
# x = 0.34 forward-most, x = 0.19 initial position
# y = 0.41 left-most, y = 0.14 right-most
# z = 0.001 lowest, z = 0.2 highest

#------------------------------------------------------------------------------------
waypoint_xyz = [
    (0.3, base_y, 0.05),
    (0.3, base_y, 0.08),
    (0.3, base_y, 0.11),
    (0.3, base_y, 0.14),
    (0.3, base_y, 0.17),
]
hold_seconds = 2.0
if not ros_bridge.wait_for_feedback(timeout_s=5.0):
    print("Warning: No /joint_states feedback received within timeout; continuing with zero-initialized non-left-arm command values.")


# Create the task composer plugin factory and load the plugins
config_path = FilesystemPath(task_composer_filename)
factory = TaskComposerPluginFactory(config_path, locator)

# Create the task composer node. In this case the FreespacePipeline is used. Many other are available.
task = factory.createTaskComposerNode("OMPLPipeline") # FreespacePipeline

# Get the output keys for the task
output_key = task.getOutputKeys().get("program")
ik = task.getInputKeys()
if ik.has("planning_input"):
    input_key = ik.get("planning_input")
elif ik.has("program"):
    input_key = ik.get("program")
else:
    raise RuntimeError("Unknown pipeline input keys")

# Create a profile dictionary. Profiles can be customized by adding to this dictionary and setting the profiles
# in the instructions.
profiles = ProfileDictionary() # profile is kept empty in this case

environment_anypoly = AnyPoly_wrap_EnvironmentConst(t_env)
profiles_anypoly = AnyPoly_wrap_ProfileDictionary(profiles)

# Create an executor to run the task
task_executor = factory.createTaskComposerExecutor("TaskflowExecutor")

current_left_arm = np.array(left_arm_pos, dtype=np.float64)

for i, (x, y, z) in enumerate(waypoint_xyz):
    # Build one segment at a time: current joint state -> Cartesian target.
    start_wp = JointWaypoint(left_arm_names, current_left_arm)
    start_instruction = MoveInstruction(
        JointWaypointPoly_wrap_JointWaypoint(start_wp),
        MoveInstructionType_FREESPACE,
        "DEFAULT",
    )
    target_wp = CartesianWaypoint(Isometry3d.Identity() * Translation3d(x, y, z) * Quaterniond(R))
    target_instruction = MoveInstruction(
        CartesianWaypointPoly_wrap_CartesianWaypoint(target_wp),
        MoveInstructionType_FREESPACE,
        "DEFAULT",
    )

    program = CompositeInstruction("DEFAULT")
    program.setManipulatorInfo(manip_info)
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(target_instruction))

    task_data = TaskComposerDataStorage()
    task_data.setData(input_key, AnyPoly_wrap_CompositeInstruction(program))
    task_data.setData("environment", environment_anypoly)
    task_data.setData("profiles", profiles_anypoly)

    future = task_executor.run(task.get(), task_data)
    future.wait()
    if not future.context.isSuccessful():
        print(f"Planning task failed at waypoint {i}: ({x:.3f}, {y:.3f}, {z:.3f})")
        exit(1)

    results = AnyPoly_as_CompositeInstruction(future.context.data_storage.getData(output_key))
    planned_states = []
    planned_times = []
    for instr in results:
        if not instr.isMoveInstruction():
            continue
        move_instr1 = InstructionPoly_as_MoveInstructionPoly(instr)
        waypoint = move_instr1.getWaypoint()
        if waypoint.isStateWaypoint():
            state_wp = WaypointPoly_as_StateWaypointPoly(waypoint)
            planned_states.append(np.array(state_wp.getPosition().flatten(), dtype=np.float64))
            planned_times.append(float(state_wp.getTime()))

    print('single intermediate state shape', planned_states[0].shape)
    print('number of planned_states', len(planned_states))

    if len(planned_states) == 0:
        print(f"No final state waypoint found at segment {i}")
        exit(1)

    # Publish each planned state (no extra interpolation), respecting planner timing.
    for state_idx, q in enumerate(planned_states):
        if state_idx > 0:
            dt = max(0.0, planned_times[state_idx] - planned_times[state_idx - 1])
            time.sleep(dt)
        ros_bridge.publish_left_arm_state(left_arm_names, q)
        viewer.update_joint_positions(left_arm_names, q)

    final_left_arm = planned_states[-1]
    current_left_arm = final_left_arm
    t_env.setState(left_arm_names, current_left_arm)
    viewer.update_joint_positions(left_arm_names, current_left_arm)
    print(f"Reached waypoint {i}: ({x:.3f}, {y:.3f}, {z:.3f}), holding {hold_seconds:.1f}s")
    time.sleep(hold_seconds)

input("press enter to exit")
ros_bridge.destroy_node()
rclpy.shutdown()