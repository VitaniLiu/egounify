from setuptools import find_packages, setup

package_name = "xynova_hand"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/xynova_hand.launch.py"]),
        (f"share/{package_name}/config", ["config/default_hand.yaml"]),
    ],
    install_requires=["setuptools", "PyYAML>=6", "litchi-hardware[xynova]>=0.1.0"],
    python_requires=">=3.10",
    zip_safe=True,
    maintainer="boilcy",
    maintainer_email="boilcy@example.com",
    description="ROS 2 command and state bridge for the Xynova Dexterous Hand.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "xynova_hand_node = xynova_hand.hand_node:main",
        ],
    },
)
