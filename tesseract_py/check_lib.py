from tesseract_robotics.tesseract_common import FilesystemPath, GeneralResourceLocator
from tesseract_robotics.tesseract_common import Isometry3d, Translation3d, Quaterniond, TransformMap
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_srdf import SRDFModel
from tesseract_robotics.tesseract_kinematics import KinematicsPluginFactory
import os
import glob
import numpy as np

# 1. FULL Environment setup (your original paths)
locator = GeneralResourceLocator()
env = Environment()
urdf_path = "/home/ahsan/ws_moveit2/src/auki_robotics_g1_humanoid_ros2/g1_description/urdf/g1_body29_hand14.urdf"
srdf_path = "/home/ahsan/ws_moveit2/src/ahsan_g1_moveit_config/config/g1.srdf"

print("Loading environment...")
env.init(FilesystemPath(urdf_path), FilesystemPath(srdf_path), locator)
print("Environment initialized:", env.isInitialized())

# 2. Get state (this was missing!)
state = env.getState()

# 3. Check SRDF groups
print("\n=== SRDF Groups ===")
groups = list(env.getGroupNames())
print(groups)

# 4. Find kinematics libraries
print("\n=== Kinematics Plugin Libraries ===")
factory = KinematicsPluginFactory()
factory.addSearchPath("/media/ahsan/T7/tesseract/tesseract_ws/install/tesseract/lib")
factory.addSearchLibrary("tesseract_trac_ik_trac-ik_factory")
libs = glob.glob("/media/ahsan/T7/tesseract/tesseract_ws/install/tesseract/lib/*tesseract*") + glob.glob("/media/ahsan/T7/tesseract/tesseract_ws/install/tesseract/lib/*kdl*factories*")


# 5. Test IK creation
print("\n=== Testing IK Solvers ===")
if groups:
    test_group = groups[0]  # Use first available group
    print(f"Testing solvers for group '{test_group}'")
    
    solvers_to_try = ["manipulator", "KDLInvKinChainLMA"]
    for solver in solvers_to_try:
        try:
            ik = factory.createInvKin(test_group, solver, env.getSceneGraph(), state)
            print(f"✅ SUCCESS: '{solver}' works for '{test_group}'!")
            break
        except Exception as e:
            print(f"❌ '{solver}' failed: {str(e)[:100]}")
else:
    print("❌ NO GROUPS LOADED - SRDF problem!")


# Test single pose solve (example waypoint)
target_pose = Isometry3d.Identity() * Translation3d(0.5, 0.0, 0.8) * Quaterniond(1, 0, 0, 0)
# Create TransformMap: {link_name: target_pose}
tip_link_poses = TransformMap()
tip_link_poses["left_wrist_yaw_link"] = target_pose  # Direct assignment!



joint_map = state.joints  # usually a dict: {joint_name: value}
joint_names = env.getGroupJointNames("left_arm")
seed_joints = [joint_map[name] for name in joint_names]
current_seed = np.array(seed_joints)

print("Seed shape:", current_seed.shape)
print("Seed values:", current_seed)

print("IK object:", ik)
try:
    names = ik.getJointNames()
    print("IK joint names:", names)
except Exception as e:
    print("IK not valid:", e)

# Now call calcInvKin with TransformMap and seed state
solutions = ik.calcInvKin(tip_link_poses, current_seed)
print(f"Found {len(solutions)} solutions")
if solutions:
    print("Joints:", solutions[0].position)