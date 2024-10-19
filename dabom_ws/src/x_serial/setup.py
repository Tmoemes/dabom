from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'x_serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # Include all Python launch files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='christopher',
    maintainer_email='christopher@karg.ca',
    description='A package for forwarding /motor_vel to /arduino_vel.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'x_serial_node = x_serial.x_serial_node:main'
        ],
    },
)
