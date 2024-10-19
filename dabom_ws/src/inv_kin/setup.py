from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'inv_kin'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),  # Include all Python launch files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='christopher',
    maintainer_email='christopher@karg.ca',
    description='Inverse kinematics for a Mecanum-wheeled robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inv_kin_node = inv_kin.inv_kin_node:main'
        ],
    },
)
