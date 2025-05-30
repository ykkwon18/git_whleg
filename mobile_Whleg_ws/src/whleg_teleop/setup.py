from setuptools import find_packages, setup

package_name = 'whleg_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lancelot22',
    maintainer_email='yk.kwon18@gmail.com',
    description='Keyboard teleoperation for ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'whleg_teleop = whleg_teleop.whleg_teleop:main',
        ],
    },
)

