from setuptools import setup, find_packages
import os
from glob import glob

package_name = "ai_flight_integrity_observer"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ZC502",
    maintainer_email="",
    description=(
        "A ROS 2 / PX4 runtime observer for flight execution integrity "
        "under AI and companion-compute load."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ai_latency_injector_node = ai_flight_integrity_observer.ai_latency_injector_node:main",
            "flight_integrity_node = ai_flight_integrity_observer.flight_integrity_node:main",
            "synthetic_px4_publisher = ai_flight_integrity_observer.synthetic_px4_publisher:main",
            "flight_diagnostics_to_csv_labeler = ai_flight_integrity_observer.flight_diagnostics_to_csv_labeler:main",
        ],
    },
)

