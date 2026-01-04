from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'amr_project'

# Recursively collect all files in models directory while preserving structure
def get_data_files(directory, install_base):
    """
    Recursively collect files from a directory and prepare them for installation
    while preserving the directory structure.
    """
    data_files = []
    
    for root, dirs, files in os.walk(directory):
        # Calculate the relative path from the source directory
        rel_path = os.path.relpath(root, directory)
        
        # Build the install path
        if rel_path == '.':
            install_path = install_base
        else:
            install_path = os.path.join(install_base, rel_path)
        
        # Collect all files in this directory
        if files:
            file_paths = [os.path.join(root, f) for f in files]
            data_files.append((install_path, file_paths))
    
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('robot_description/launch/*.launch.py')),
        
        # URDF files
        (os.path.join('share', package_name, 'urdf'), 
            glob('robot_description/urdf/*.urdf') + 
            glob('robot_description/urdf/*.xacro')),
        
        # Config files
        (os.path.join('share', package_name, 'config'), 
            glob('robot_description/config/*.yaml')),
        
        # RViz files
        (os.path.join('share', package_name, 'rviz'), 
            glob('robot_description/rviz/*.rviz')),
        
        # Meshes
        (os.path.join('share', package_name, 'meshes'), 
            glob('robot_description/meshes/*.STL') +
            glob('robot_description/meshes/*.stl') +
            glob('robot_description/meshes/*.dae') +
            glob('robot_description/meshes/*.obj')),
        
        # Worlds
        (os.path.join('share', package_name, 'worlds'), 
            glob('robot_description/worlds/*.world') +
            glob('robot_description/worlds/*.sdf')),
        
        # Photos
        (os.path.join('share', package_name, 'photos'), 
            glob('robot_description/photos/*.jpg') +
            glob('robot_description/photos/*.png')),
        
    ] + get_data_files('robot_description/models', os.path.join('share', package_name, 'models')),
    
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