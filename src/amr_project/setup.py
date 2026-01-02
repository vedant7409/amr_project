from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'amr_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('robot_description/launch/*.launch.py')),
        # Add URDF files
        (os.path.join('share', package_name, 'urdf'), 
            glob('robot_description/urdf/*.urdf') + 
            glob('robot_description/urdf/*.xacro')),
        # Add config files
        (os.path.join('share', package_name, 'config'), 
            glob('robot_description/config/*.yaml')),
        # Add RViz config files
        (os.path.join('share', package_name, 'rviz'), 
            glob('robot_description/rviz/*.rviz')),
        # Add meshes
        (os.path.join('share', package_name, 'meshes'), 
            glob('robot_description/meshes/*.STL') +
            glob('robot_description/meshes/*.stl') +
            glob('robot_description/meshes/*.dae') +
            glob('robot_description/meshes/*.obj')),
        # Add photos (textures for Gazebo models)
        (os.path.join('share', package_name, 'photos'), 
            glob('robot_description/photos/*.jpg') +
            glob('robot_description/photos/*.png')),
        # Add Gazebo models (AWS RoboMaker models)
        (os.path.join('share', package_name, 'models'), 
            glob('robot_description/models/**/*', recursive=True)),
        # Add world files for Gazebo
        (os.path.join('share', package_name, 'worlds'), 
            glob('robot_description/worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vedant',
    maintainer_email='vedantkorgaonkar7@gmail.com',
    description='AMR SLAM Project for Orin Nano',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Add your ROS 2 nodes here when you create them
            # Example: 'slam_node = amr_project.slam_node:main',
        ],
    },
)