from setuptools import setup

package_name = 'nucleus_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'clients', 'subscribers'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='martin',
    maintainer_email='martin.johansen@nortekgroup.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nucleus_node = nucleus_node.nucleus_node:main',

            'connect_tcp = clients.connect_tcp:main',
            'connect_serial = clients.connect_serial:main',
            'disconnect = clients.disconnect:main',
            'start = clients.start:main',
            'field_calibration = clients.field_calibration:main',
            'stop = clients.stop:main',
            'command = clients.command:main',

            'ahrs_packets = subscribers.ahrs_packets:main',
            'altimeter_packets = subscribers.altimeter_packets:main',
            'bottom_track_packets = subscribers.bottom_track_packets:main',
            'current_profile_packets = subscribers.current_profile_packets:main',
            'field_calibration_packets = subscribers.field_calibration_packets:main',
            'imu_packets = subscribers.imu_packets:main',
            'ins_packets = subscribers.ins_packets:main',
            'mag_packets = subscribers.mag_packets:main',
            'water_track_packets = subscribers.water_track_packets:main',
        ],
    },
)
