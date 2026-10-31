import os
from glob import glob

from setuptools import find_packages, setup

package_name = "manus_wuji_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=[
        "numpy>=1.26,<3",
        "setuptools",
        "wuji-sdk>=2026.8.3,<2026.9",
    ],
    python_requires=">=3.10",
    zip_safe=True,
    maintainer="boilcy",
    maintainer_email="boilcy@example.com",
    description="MANUS glove to Wuji Hand 2 teleoperation using official retargeting.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "retarget_node = manus_wuji_teleop.retarget_node:main",
        ],
    },
)
