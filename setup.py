import os
from glob import glob

from setuptools import setup

package_name = "g1_solver"


def collect_description_data_files():
    data = []
    desc = "description"
    if not os.path.isdir(desc):
        return data
    for root, _, files in os.walk(desc):
        if not files:
            continue
        rel = os.path.relpath(root, desc)
        dst = os.path.join("share", package_name, "description", rel if rel != "." else "")
        dst = os.path.normpath(dst)
        data.append((dst, [os.path.join(root, f) for f in sorted(files)]))
    return data


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_dir={package_name: "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob(os.path.join("launch", "*.py"))),
        (os.path.join("share", package_name, "config"), glob(os.path.join("config", "*"))),
    ]
    + collect_description_data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ahsan Ahmed",
    maintainer_email="ahsan@scaledrive.ai",
    description="Tesseract G1 arm solver ROS 2 node",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "g1_solver_main = g1_solver.main:main",
            "g1_solver_mock_joint_states = g1_solver.publish_joint_states:main",
        ],
    },
)
