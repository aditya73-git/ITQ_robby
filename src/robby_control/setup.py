from setuptools import find_packages, setup


package_name = "robby_control"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/ackermann_cmd.yaml"]),
        (f"share/{package_name}/config", ["config/ackermann_controller_bridge.yaml"]),
        (f"share/{package_name}/config", ["config/ackermann_joint_state_bridge.yaml"]),
        (f"share/{package_name}/config", ["config/four_wis4wid_cmd.yaml"]),
        (f"share/{package_name}/config", ["config/swerve_odometry.yaml"]),
        (f"share/{package_name}/launch", ["launch/ackermann_visualization.launch.py"]),
        (f"share/{package_name}/launch", ["launch/ackermann.launch.py"]),
        (f"share/{package_name}/launch", ["launch/four_wis4wid.launch.py"]),
        (f"share/{package_name}/launch", ["launch/odometry.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aditya",
    maintainer_email="kotteaditya919@gmail.com",
    description="Control and odometry nodes for the ITQ Robby platform.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "ackermann_cmd_node = robby_control.ackermann_cmd_node:main",
            "ackermann_controller_bridge = robby_control.ackermann_controller_bridge:main",
            "ackermann_joint_state_bridge = robby_control.ackermann_joint_state_bridge:main",
            "four_wis4wid_cmd_node = robby_control.four_wis4wid_cmd_node:main",
            "swerve_odometry_node = robby_control.swerve_odometry_node:main",
        ],
    },
)
