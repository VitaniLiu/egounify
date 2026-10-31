from setuptools import find_packages, setup

package_name = "sharpa_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
    ],
    install_requires=["setuptools", "litchi-hardware[sharpa]>=0.1.4"],
    python_requires=">=3.10",
    zip_safe=True,
    maintainer="boilcy",
    maintainer_email="boilcy@example.com",
    description="ROS 2 command and state bridge for Sharpa Wave hands.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "sharpa_driver = sharpa_driver.node:main",
        ],
    },
)
