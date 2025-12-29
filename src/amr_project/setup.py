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
            glob('robot_description/urdf/*')),
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
        ],
    },
)