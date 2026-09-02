from glob import glob
from setuptools import find_packages, setup

package_name = "inspection_rerun"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    scripts=["scripts/rerun_bridge"],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Tank Inspection Maintainer",
    maintainer_email="maintainer@example.com",
    description="Optional ROS 2 to Rerun visualization bridge for inspection MCAP replay.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
