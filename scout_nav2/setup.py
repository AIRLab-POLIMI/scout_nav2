from setuptools import setup
import os
from glob import glob

package_name = 'scout_nav2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
		(os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
		(os.path.join('share', package_name, "maps"), glob('maps/map_*')),
		(os.path.join('share', package_name, "params"), glob('params/*.yaml')),
		(os.path.join('share', package_name, "rviz"), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='simongiampa',
    maintainer_email='simonegiampa99@gmail.com',
    description='Nav2 package for localization, navigation, SLAM and more',
    license='AIRLAB',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_navigate_to_pose = siri_nav2.example_navigate_to_pose:main'
        ],
    },
)
