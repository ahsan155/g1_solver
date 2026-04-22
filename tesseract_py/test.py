from tesseract_robotics.tesseract_common import FilesystemPath, GeneralResourceLocator, ManipulatorInfo
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_srdf import SRDFModel
from tesseract_robotics.tesseract_kinematics import KinematicsPluginFactory

env = Environment()
locator = GeneralResourceLocator()

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

# locator_fn must be kept alive by maintaining a reference
assert t_env.init(abb_irb2400_urdf_fname, abb_irb2400_srdf_fname, locator)