from tesseract_robotics.tesseract_common import FilesystemPath, GeneralResourceLocator, ManipulatorInfo
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_srdf import SRDFModel
from tesseract_robotics.tesseract_kinematics import KinematicsPluginFactory

env = Environment()
locator = GeneralResourceLocator()

urdf_path = "/home/ahsan/ws_moveit2/src/auki_robotics_g1_humanoid_ros2/g1_description/urdf/g1_body29_hand14.urdf"
srdf_path = "/home/ahsan/ws_moveit2/src/ahsan_g1_moveit_config/config/g1.srdf"

env.init(FilesystemPath(urdf_path), FilesystemPath(srdf_path), locator)
links = list(env.getLinkNames())

#print("Initialized:", env.isInitialized())
#print(env.getLinkNames())
#print(links)

manip = ManipulatorInfo()
manip.manipulator = "left_arm"
manip.tcp_frame = "left_wrist_yaw_link"
manip.working_frame = "torso_link"


factory = KinematicsPluginFactory()
factory.addSearchPath("/opt/ros/humble/lib")
factory.addSearchLibrary("tesseract_kinematics_kdl_factories")

state = env.getState()
groups = list(env.getGroupNames())
print(groups)


ik = factory.createInvKin(
    "left_arm",
    "KDLInvKinChainLMA",
    env.getSceneGraph(),
    state
)

