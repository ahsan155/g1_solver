# minimized code and added more waypoints
# spending more time after reaching a waypoint

import re
import traceback
import os
import time
import numpy as np
import numpy.testing as nptest

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

task_composer_filename = os.environ["TESSERACT_TASK_COMPOSER_CONFIG_FILE"]

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
frame_dt = 0.04
interp_steps = 6


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
    for instr in results:
        if not instr.isMoveInstruction():
            continue
        move_instr1 = InstructionPoly_as_MoveInstructionPoly(instr)
        waypoint = move_instr1.getWaypoint()
        if waypoint.isStateWaypoint():
            state_wp = WaypointPoly_as_StateWaypointPoly(waypoint)
            planned_states.append(np.array(state_wp.getPosition().flatten(), dtype=np.float64))

    print('planned_states shape', planned_states[0].shape)
    print('size of planned_states', len(planned_states))

    if len(planned_states) == 0:
        print(f"No final state waypoint found at segment {i}")
        exit(1)

    # Smooth playback in viewer to avoid trajectory reload/snap effect.
    for state_idx in range(len(planned_states) - 1):
        q0 = planned_states[state_idx]
        q1 = planned_states[state_idx + 1]
        for step in range(1, interp_steps + 1):
            alpha = step / float(interp_steps)
            q = (1.0 - alpha) * q0 + alpha * q1
            viewer.update_joint_positions(left_arm_names, q)
            time.sleep(frame_dt)

    final_left_arm = planned_states[-1]
    current_left_arm = final_left_arm
    t_env.setState(left_arm_names, current_left_arm)
    viewer.update_joint_positions(left_arm_names, current_left_arm)
    print(f"Reached waypoint {i}: ({x:.3f}, {y:.3f}, {z:.3f}), holding {hold_seconds:.1f}s")
    time.sleep(hold_seconds)

input("press enter to exit")