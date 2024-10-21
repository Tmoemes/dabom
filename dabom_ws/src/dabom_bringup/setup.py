from setuptools import setup
from glob import glob
import os

package_name = 'dabom_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],  # No Python packages to install
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include all config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # Include any RViz configuration files
        (os.path.join('share', package_name, 'config'), glob('config/*.rviz')),
        # Include any maps (if you have a maps directory)
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christopher',
    maintainer_email='christopher@karg.ca',
    description='Master launch package for bringing up the entire robot system',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # No console scripts in this package
        ],
    },
)
