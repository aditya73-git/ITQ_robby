from setuptools import find_packages, setup


package_name = "robby_localization"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/ekf.yaml"]),
        (f"share/{package_name}/launch", ["launch/localization.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="aditya",
    maintainer_email="kotteaditya919@gmail.com",
    description="Localization and sensor fusion nodes for the ITQ Robby platform.",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "imu_preprocessor_node = robby_localization.imu_preprocessor_node:main",
        ],
    },
)
