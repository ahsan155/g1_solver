from tesseract_robotics.tesseract_common import FilesystemPath, GeneralResourceLocator, ManipulatorInfo
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_srdf import SRDFModel
from tesseract_robotics.tesseract_kinematics import KinematicsPluginFactory
from tesseract_robotics.tesseract_collision import ContactResultMap, ContactRequest, ContactResultVector, ContactTestType_ALL

import numpy as np


# export TESSERACT_RESOURCE_PATH=/media/ahsan/T7/tesseract/tesseract_ws/tesseract_python/tesseract_python
#config_package_url = "package://tesseract_python/examples/tesseract_resource/config/g1_plugins.yaml"
#config_plugins_fname = FilesystemPath(locator.locateResource(config_package_url).getFilePath())
#print('config file path:', config_plugins_fname)

# export TESSERACT_RESOURCE_PATH=/media/ahsan/T7/tesseract/tesseract_ws/tesseract_python
locator = GeneralResourceLocator()
abb_irb2400_urdf_package_url = "package://tesseract/mycode/resource/g1.urdf"
abb_irb2400_srdf_package_url = "package://tesseract/mycode/resource/g1.srdf"
abb_irb2400_urdf_fname = FilesystemPath(locator.locateResource(abb_irb2400_urdf_package_url).getFilePath())
abb_irb2400_srdf_fname = FilesystemPath(locator.locateResource(abb_irb2400_srdf_package_url).getFilePath())
print('urdf path', abb_irb2400_urdf_fname)
print('srdf path', abb_irb2400_srdf_fname)

t_env = Environment()
locator = GeneralResourceLocator()


# locator_fn must be kept alive by maintaining a reference
assert t_env.init(abb_irb2400_urdf_fname, abb_irb2400_srdf_fname, locator)

all_names = list(t_env.getActiveJointNames())   # or union of left/right/waist/hand joints
all_pos = np.zeros(len(all_names))


robot_joint_pos = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.0])
robot_joint_names = list(t_env.getGroupJointNames("right_arm"))


solver = t_env.getStateSolver()
manager = t_env.getDiscreteContactManager()
manager.setActiveCollisionObjects(t_env.getActiveLinkNames())

solver.setState(all_names, all_pos)
scene_state = solver.getState()
manager.setCollisionObjectsTransform(scene_state.link_transforms)

contact_result_map = ContactResultMap()
manager.contactTest(contact_result_map, ContactRequest(ContactTestType_ALL))
result_vector = ContactResultVector()
contact_result_map.flattenMoveResults(result_vector)

print(f"Found {len(result_vector)} contact results")
for i in range(len(result_vector)):
    contact_result = result_vector[i]
    print(f"Contact {i}:")
    print(f"\tDistance: {contact_result.distance}")
    print(f"\tLink A: {contact_result.link_names[0]}")
    print(f"\tLink B: {contact_result.link_names[1]}")


xx = ['aa', 'bb']
yy = ['aa', 'bb', 'cc']

zz = [x if x in yy for x in xx]
print(zz)