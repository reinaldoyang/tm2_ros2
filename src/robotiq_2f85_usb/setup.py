from setuptools import setup

package_name = "robotiq_2f85_usb"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", "pyRobotiqGripper"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="Minimal ROS 2 USB driver for Robotiq 2F-85",
    license="MIT",
    entry_points={
        "console_scripts": [
            "robotiq_2f85_usb_node = robotiq_2f85_usb.node:main",
        ],
    },
)
