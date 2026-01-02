from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'amr_project'

# Helper function to capture nested directory structures (important for AWS models)
def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            # We want to install to share/package_name/path_within_src
            # path is something like 'robot_description/models/aws_warehouse/meshes'
            install_path = os.path.join('share', package_name, path)
            paths.append((install_path, [os.path.join(path, filename)]))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Standard folders
        (os.path.join('share', package_name, 'launch'), glob('robot_description/launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('robot_description/urdf/*.urdf') + glob('robot_description/urdf/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('robot_description/config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('robot_description/rviz/*.rviz')),
        (os.path.join('share', package_name, 'meshes'), glob('robot_description/meshes/*')),
        (os.path.join('share', package_name, 'worlds'), glob('robot_description/worlds/*.world')),
        (os.path.join('share', package_name, 'photos'), glob('robot_description/photos/*')),
        
    ] + package_files('robot_description/models'), 
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vedant',
    maintainer_email='vedantkorgaonkar7@gmail.com',
    description='AMR SLAM Project for Orin Nano',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)