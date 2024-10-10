from setuptools import find_packages, setup

package_name = 'serial_comm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['serial_comm/launch/serial_talker_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mees',
    maintainer_email='38130223+Tmoemes@users.noreply.github.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_talker = serial_comm.serial_talker:main',
        ],
    },
)
