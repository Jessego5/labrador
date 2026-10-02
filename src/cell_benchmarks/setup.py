"""
This is the ament_python package definition for cell_benchmarks. It installs the scene
randomiser and the three acceptance runners, and registers run_perception_trials,
run_insertion_trials and run_task_trials as console scripts so each one starts with ros2 run
cell_benchmarks <name>. Build it with colcon build --packages-select cell_benchmarks.
"""
from setuptools import find_packages, setup

package_name = "cell_benchmarks"

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
    description="Scene randomization and benchmark runners for the manipulation cell.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "run_perception_trials = cell_benchmarks.run_perception_trials:main",
            "run_insertion_trials = cell_benchmarks.run_insertion_trials:main",
            "run_task_trials = cell_benchmarks.run_task_trials:main",
            "run_lab_trials = cell_benchmarks.run_lab_trials:main",
        ],
    },
)
