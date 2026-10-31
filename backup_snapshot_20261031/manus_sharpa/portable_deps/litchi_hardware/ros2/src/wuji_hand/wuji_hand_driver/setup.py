from glob import glob

from setuptools import find_packages, setup

package_name = "wuji_hand_driver"

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
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools", "wuji-sdk>=2026.8.3,<2026.9"],
    extras_require={"test": ["pytest"]},
    python_requires=">=3.10",
    zip_safe=True,
    maintainer="boilcy",
    maintainer_email="boilcy@example.com",
    description="ROS 2 command and state bridge for Wuji Hand 2.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "wuji-hand-2-driver = wuji_hand_driver.node:main",
        ],
    },
)
