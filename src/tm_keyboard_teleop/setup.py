from setuptools import setup

package_name = "tm_keyboard_teleop"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="Keyboard teleop for TM robot and Robotiq gripper",
    license="MIT",
    entry_points={
        "console_scripts": [
            "tm_keyboard_teleop = tm_keyboard_teleop.tm_keyboard_teleop:main",
        ],
    },
)
