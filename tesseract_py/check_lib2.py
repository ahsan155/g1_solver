import re
import traceback
import os
import numpy as np
import numpy.testing as nptest

from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_environment import Environment, AnyPoly_wrap_EnvironmentConst
from tesseract_robotics.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond, \
    ManipulatorInfo, AnyPoly, AnyPoly_wrap_double
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

tesseract_common.setLogLevel(tesseract_common.CONSOLE_BRIDGE_LOG_DEBUG)

OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask"
TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"

task_composer_filename = os.environ["TESSERACT_TASK_COMPOSER_CONFIG_FILE"]
print('yy', task_composer_filename)

# Initialize the resource locator and environment
locator = GeneralResourceLocator()
abb_irb2400_urdf_package_url = "package://tesseract/support/urdf/g1.urdf"
abb_irb2400_srdf_package_url = "package://tesseract/support/urdf/g1.srdf"
abb_irb2400_urdf_fname = FilesystemPath(locator.locateResource(abb_irb2400_urdf_package_url).getFilePath())
abb_irb2400_srdf_fname = FilesystemPath(locator.locateResource(abb_irb2400_srdf_package_url).getFilePath())
print('xx', abb_irb2400_urdf_fname)

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

print(list(t_env.getGroupJointNames("left_arm")))

# Set the initial state of the robot
joint_names = list(t_env.getGroupJointNames("left_arm"))
viewer.update_joint_positions(joint_names, np.array([0.0,0.0,0.0,0.0,0.0,0.0,0.0]))

# Start the viewer
viewer.start_serve_background()

# Set the initial state of the robot
t_env.setState(joint_names, np.ones(7)*0.0)

jg = t_env.getKinematicGroup("left_arm")

#-----------------------
# Create the input command program waypoints
#wp1 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8,-0.3,1.455) * Quaterniond(0.70710678,0,0.70710678,0))
wp1 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.25,-0.15,1.15) * Quaterniond(0.70710678,0,0.70710678,0))
wp2 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.30,-0.10,1.20) * Quaterniond(0.70710678,0,0.70710678,0))
wp3 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8,0.5,1.455) * Quaterniond(0.70710678,0,0.70710678,0))

# Create the input command program instructions. Note the use of explicit construction of the CartesianWaypointPoly
# using the *_wrap_CartesianWaypoint functions. This is required because the Python bindings do not support implicit
# conversion from the CartesianWaypoint to the CartesianWaypointPoly.
#start_instruction = MoveInstruction(CartesianWaypointPoly_wrap_CartesianWaypoint(wp1), MoveInstructionType_FREESPACE, "DEFAULT")
start_wp = JointWaypoint(joint_names, np.zeros(7))

start_instruction = MoveInstruction(
    JointWaypointPoly_wrap_JointWaypoint(start_wp),
    MoveInstructionType_FREESPACE,
    "DEFAULT"
)
plan_f1 = MoveInstruction(CartesianWaypointPoly_wrap_CartesianWaypoint(wp2), MoveInstructionType_FREESPACE, "DEFAULT")
plan_f2 = MoveInstruction(CartesianWaypointPoly_wrap_CartesianWaypoint(wp3), MoveInstructionType_FREESPACE, "DEFAULT")

# Create the input command program. Note the use of *_wrap_MoveInstruction functions. This is required because the
# Python bindings do not support implicit conversion from the MoveInstruction to the MoveInstructionPoly.
program = CompositeInstruction("DEFAULT") # DEFAULT
program.setManipulatorInfo(manip_info)
program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
# program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))
# program.appendMoveInstruction(MoveInstructionPoly(plan_f2))

# Create the task composer plugin factory and load the plugins
config_path = FilesystemPath(task_composer_filename)
factory = TaskComposerPluginFactory(config_path, locator)

# Create the task composer node. In this case the FreespacePipeline is used. Many other are available.
task = factory.createTaskComposerNode("FreespacePipeline") # FreespacePipeline

# Get the output keys for the task
output_key = task.getOutputKeys().get("program")
input_key = task.getInputKeys().get("planning_input")

# Create a profile dictionary. Profiles can be customized by adding to this dictionary and setting the profiles
# in the instructions.
profiles = ProfileDictionary()

# Create an AnyPoly containing the program. This explicit step is required because the Python bindings do not
# support implicit conversion from the CompositeInstruction to the AnyPoly.
program_anypoly = AnyPoly_wrap_CompositeInstruction(program)
environment_anypoly = AnyPoly_wrap_EnvironmentConst(t_env)
profiles_anypoly = AnyPoly_wrap_ProfileDictionary(profiles)

# Create the task data
task_data = TaskComposerDataStorage()
task_data.setData(input_key, program_anypoly)
task_data.setData("environment", environment_anypoly)
task_data.setData("profiles", profiles_anypoly)


# Create an executor to run the task
task_executor = factory.createTaskComposerExecutor("TaskflowExecutor")

# Run the task and wait for completion
future = task_executor.run(task.get(), task_data)
future.wait()

if not future.context.isSuccessful():
    print("Planning task failed")
    exit(1)

# Retrieve the output, converting the AnyPoly back to a CompositeInstruction
results = AnyPoly_as_CompositeInstruction(future.context.data_storage.getData(output_key))

# Display the output
# Print out the resulting waypoints
for instr in results:
    assert instr.isMoveInstruction()
    move_instr1 = InstructionPoly_as_MoveInstructionPoly(instr)
    wp1 = move_instr1.getWaypoint()
    assert wp1.isStateWaypoint()
    wp = WaypointPoly_as_StateWaypointPoly(wp1)
    print(f"Joint Positions: {wp.getPosition().flatten()} time: {wp.getTime()}")

# Update the viewer with the results to animate the trajectory
# Open web browser to http://localhost:8000 to view the results
viewer.update_trajectory(results)
viewer.plot_trajectory(results, manip_info)

input("press enter to exit")