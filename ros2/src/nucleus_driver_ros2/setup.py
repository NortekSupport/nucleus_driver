from setuptools import setup

package_name = "nucleus_driver_ros2"

setup(
    name=package_name,
    version="0.2.1",
    packages=["nucleus_node", "nucleus_clients", "nucleus_subscribers"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="martin",
    maintainer_email="martin.johansen@nortekgroup.com",
    description="This is a ros2 packege that builds on the Nucleus Driver. The pacakge consists of the nucleus_node which maps the functionality of the nucleus driver to a ros2 interface allowing communication to a Nucleus device over a ros2 environment",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "nucleus_node = nucleus_node.nucleus_node:main",
            "connect_tcp = nucleus_clients.connect_tcp:main",
            "connect_serial = nucleus_clients.connect_serial:main",
            "disconnect = nucleus_clients.disconnect:main",
            "start = nucleus_clients.start:main",
            "field_calibration = nucleus_clients.field_calibration:main",
            "stop = nucleus_clients.stop:main",
            "command = nucleus_clients.command:main",
            "ahrs_packets = nucleus_subscribers.ahrs_packets:main",
            "altimeter_packets = nucleus_subscribers.altimeter_packets:main",
            "bottom_track_packets = nucleus_subscribers.bottom_track_packets:main",
            "current_profile_packets = nucleus_subscribers.current_profile_packets:main",
            "field_calibration_packets = nucleus_subscribers.field_calibration_packets:main",
            "imu_packets = nucleus_subscribers.imu_packets:main",
            "ins_packets = nucleus_subscribers.ins_packets:main",
            "magnetometer_packets = nucleus_subscribers.magnetometer_packets:main",
            "water_track_packets = nucleus_subscribers.water_track_packets:main",
        ],
    },
)
