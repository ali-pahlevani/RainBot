import os
from glob import glob
from setuptools import setup

package_name = 'pickplace_arm_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.rviz') + glob('config/*.xml')),
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*.yaml') + glob('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ali Pahlevani',
    maintainer_email='a.pahlevani1998@gmail.com',
    description='Pick and place automation for pickplace_arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_pickPlace = pickplace_arm_bringup.mission_2:main_tugbot',
            'mission_rackPlace = pickplace_arm_bringup.mission_2:main_hospital',
            'aws_hospital_map = pickplace_arm_bringup.aws_hospital_map:main',
            'nav_bringup = pickplace_arm_bringup.nav_bringup:main',
            'rack_release = pickplace_arm_bringup.rack_release:main',
            'map_pump = pickplace_arm_bringup.map_pump:main',
            'mission_delivery = pickplace_arm_bringup.mission_delivery:main',
            'task_manager = pickplace_arm_bringup.task_manager:main',
            # Manual driving, used when building a map with mapping.launch.py.
            'teleop_key = pickplace_arm_bringup.teleop_key:main',
            # Startup readiness gate used by mission_pickPlace.launch.py.
            'wait_for = pickplace_arm_bringup.wait_for:main',
        ],
    },
)
