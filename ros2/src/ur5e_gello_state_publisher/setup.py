from setuptools import find_packages, setup
import glob

package_name = "ur5e_gello_state_publisher"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob.glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob.glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pyserial"],
    zip_safe=True,
    maintainer="GELLO Project",
    maintainer_email="user@example.com",
    description="Publishes STS3215 GELLO joint readings as ROS 2 UR5e joint commands.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fake_ur5e_gello_publisher = ur5e_gello_state_publisher.fake_ur5e_gello_publisher:main",
            "rg6_tool_tcp_node = ur5e_gello_state_publisher.rg6_tool_tcp_node:main",
            "ur5e_gello_publisher = ur5e_gello_state_publisher.ur5e_gello_publisher:main",
        ],
    },
)
