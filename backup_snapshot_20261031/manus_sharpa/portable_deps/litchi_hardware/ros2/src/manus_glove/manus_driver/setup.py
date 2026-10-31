from setuptools import find_packages, setup

package_name = "manus_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="boilcy",
    maintainer_email="boilcy@example.com",
    description="ROS 2 transport bridge for the litchi_hardware Manus tracker.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "manus_driver = manus_driver.node:main",
        ],
    },
)
