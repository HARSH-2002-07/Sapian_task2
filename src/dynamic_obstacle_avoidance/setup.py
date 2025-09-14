from setuptools import setup
import os
from glob import glob

package_name = 'dynamic_obstacle_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add launch files
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Dynamic obstacle avoidance for TurtleBot3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_spawner = dynamic_obstacle_avoidance.obstacle_spawner:main',
            'simple_controller = dynamic_obstacle_avoidance.simple_controller:main',
        ],
    },
)