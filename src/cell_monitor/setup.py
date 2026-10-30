"""
This is the ament_python package definition for cell_monitor. It installs the fault logic and
the monitor node and registers monitor_node as a console script, so it starts with ros2 run
cell_monitor monitor_node (cell.launch.py starts it itself). Build it with colcon build
--packages-select cell_monitor.
"""
from setuptools import find_packages, setup

package_name = "cell_monitor"

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
    description="Fault injection for the cell's benchmarks.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["monitor_node = cell_monitor.monitor_node:main"]},
)
