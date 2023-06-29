from setuptools import setup

package_name = 'nucleus_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
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
            'connect_tcp = nucleus_node.client_connect_tcp:main',
            'connect_serial = nucleus_node.client_connect_serial:main',
            'disconnect = nucleus_node.client_disconnect:main',
            'start = nucleus_node.client_start:main',
            'stop = nucleus_node.client_stop:main',
            'read_packet = nucleus_node.client_read_packet:main',
        ],
    },
)
