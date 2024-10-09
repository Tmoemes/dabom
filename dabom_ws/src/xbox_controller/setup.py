from setuptools import setup

package_name = 'xbox_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=['xbox_controller_node'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pygame', 'inputs'],
    zip_safe=True,
    maintainer='CKarg',
    maintainer_email='your.email@example.com',
    description='Xbox controller ROS2 node',
    license='License declaration',
    entry_points={
        'console_scripts': [
            'xbox_controller_node = xbox_controller_node:main',
        ],
    },
)
