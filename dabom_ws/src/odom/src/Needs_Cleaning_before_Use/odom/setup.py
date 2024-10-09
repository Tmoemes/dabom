from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'odom'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),  # Automatically find all packages in the source directory
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christopher Karg',
    maintainer_email='christopher@karg.ca',
    description='A package for calculating odometry for a Mecanum-wheeled robot',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'odom_node = odom.odom_node:main',  # Entry point for the main odometry script
        ],
    },
    data_files=[
        (os.path.join('share', package_name), ['package.xml']),  # Install package.xml
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),  # Install launch files
        (os.path.join('share/ament_index/resource_index/packages'), ['resource/odom']),  # Install a marker in the package index
    ],
)
