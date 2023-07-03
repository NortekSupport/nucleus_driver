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
            'connect_tcp_old = nucleus_node.client_connect_tcp:main',
            'connect_serial_old = nucleus_node.client_connect_serial:main',
            'disconnect_old = nucleus_node.client_disconnect:main',
            'start_old = nucleus_node.client_start:main',
            'stop_old = nucleus_node.client_stop:main',
            'read_packet_old = nucleus_node.client_read_packet:main',
            'command_old = nucleus_node.client_command:main',
            'ahrs_packets_old = nucleus_node.subscriber_ahrs_packets:main',

            'connect_tcp = clients.connect_tcp:main',
            'connect_serial = clients.connect_serial:main',
            'disconnect = clients.disconnect:main',
            'start = clients.start:main',
            'stop = clients.stop:main',
            'read_packet = clients.read_packet:main',
            'command = clients.command:main',

            'ahrs_packets = subscribers.ahrs_packets:main',
        ],
    },
)
