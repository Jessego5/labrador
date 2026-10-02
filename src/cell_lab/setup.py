"""
This is the ament_python package definition for cell_lab. It installs the chemistry and the
lab node and registers lab_node as a console script, so it starts with ros2 run cell_lab
lab_node (cell.launch.py starts it itself with scene:=lab). Build it with colcon build
--packages-select cell_lab.
"""
from setuptools import find_packages, setup

package_name = "cell_lab"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cell developer",
    maintainer_email="dev@example.com",
    description="The lab scene's chemistry, shown in MuJoCo by swapping visual twins.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["lab_node = cell_lab.lab_node:main"]},
)
