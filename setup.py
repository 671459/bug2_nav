from setuptools import setup
import os
from glob import glob

package_name = 'bug2_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Havard',
    maintainer_email='671459@stud.hvl.no',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wall_follower = bug2_navigation.wall_follower:main',
            'go_to_point = bug2_navigation.go_to_point:main',
            'WFservice = bug2_navigation.wall_follower_service:main',
            'GTPservice = bug2_navigation.go_to_point_service:main',
            'WFclient = bug2_navigation.wall_follower_client:main',
            'GTPclient = bug2_navigation.go_to_point_client:main',
            'bug2_controller = bug2_navigation.bug2_controller:main',
        ],
    },
)