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

#tesseract_common.setLogLevel(tesseract_common.CONSOLE_BRIDGE_LOG_DEBUG)

OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask"
TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"

task_composer_filename = os.environ["TESSERACT_TASK_COMPOSER_CONFIG_FILE"]

# Initialize the resource locator and environment
locator = GeneralResourceLocator()
abb_irb2400_urdf_package_url = "package://tesseract/support/urdf/g1_urdf_new.urdf"
abb_irb2400_srdf_package_url = "package://tesseract/support/urdf/g1_srdf_new.srdf"
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

joint_names = list(t_env.getGroupJointNames("left_arm"))

viewer.update_joint_positions(joint_names, np.array([0.1,0.1,0.1,0.1,0.1,0.1,0.0]))
viewer.start_serve_background()


# Initial pose
q = np.array([0.1,0.1,0.1,0.1,0.1,0.1,0.0])
print('start....')

for i in range(200):

    if i < 100:
        q[0] += 0.005      # shoulder motion
        q[3] -= 0.01       # elbow bend
    else:
        q[3] += 0.01
    
    viewer.update_joint_positions(joint_names, q)

    time.sleep(0.05)

viewer.update_joint_positions(joint_names, np.array([0.1,0.1,0.1,0.1,0.1,0.1,0.0]))
for i in range(100):

    
    q[0] -= 0.005      # shoulder motion
    q[3] -= 0.01 

    viewer.update_joint_positions(joint_names, q)

    time.sleep(0.05)