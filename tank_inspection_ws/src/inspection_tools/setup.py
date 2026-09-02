from setuptools import find_packages, setup

package_name = "inspection_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="Tank Inspection Maintainer",
    maintainer_email="maintainer@example.com",
    description="Validation and immutable finalization tools for inspection MCAP bags.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "validate_bag = inspection_tools.validate_bag:main",
            "finalize_bag = inspection_tools.finalize_bag:main",
            "estimate_storage = inspection_tools.estimate_storage:main",
            "depth_to_pointcloud = inspection_tools.depth_to_pointcloud:main",
            "inspect_latest = inspection_tools.inspect_latest:main",
        ],
    },
)
