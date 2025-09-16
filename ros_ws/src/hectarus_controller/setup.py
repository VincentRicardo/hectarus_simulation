import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'hectarus_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cent',
    maintainer_email='cent@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "hectarus_teleop_key = hectarus_controller.hectarus_teleop_key:main",
            "gazebo_joint_publisher = hectarus_controller.gazebo_joint_publisher:main",
            "hectarus_torque_data = hectarus_controller.hectarus_torque_data:main",
            "hectarus_imu_data = hectarus_controller.hectarus_imu_data:main"
        ],
    },
)
