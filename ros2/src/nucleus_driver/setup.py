from setuptools import setup

package_name = 'nucleus_driver'

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
    description='Nucleus driver',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nucleus_node = nucleus_driver.nucleus_node:main'
        ],
    },
)
