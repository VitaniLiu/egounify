from glob import glob

from setuptools import find_packages, setup

package_name = "linker_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=[
        "setuptools",
        "PyYAML>=6",
        "litchi-hardware[linker]>=0.1.0",
    ],
    python_requires=">=3.10",
    zip_safe=True,
    maintainer="boilcy",
    maintainer_email="boilcy@example.com",
    description="ROS 2 bridge for Linker Hand G20 and O30I devices.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "linker_hand_node = linker_driver.hand_node:main",
        ],
    },
)
