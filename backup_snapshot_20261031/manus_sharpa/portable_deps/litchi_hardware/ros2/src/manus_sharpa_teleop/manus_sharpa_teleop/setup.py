import os
from glob import glob

from setuptools import find_packages, setup

package_name = "manus_sharpa_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (
            os.path.join("share", package_name, "retarget_v4", "include"),
            glob("vendor/retarget_v4/include/*.so"),
        ),
        (
            os.path.join("share", package_name, "retarget_v4"),
            glob("vendor/retarget_v4/*.json")
            + glob("vendor/retarget_v4/*.vendor"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="boilcy",
    maintainer_email="boilcy@example.com",
    description="Launch-ready Manus to Sharpa Wave teleoperation pipeline.",
    license="Apache-2.0 AND LicenseRef-Proprietary",
    entry_points={
        "console_scripts": [
            "retarget_node = manus_sharpa_teleop.retarget_node:main",
        ],
    },
)
