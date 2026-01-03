from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    bot_description_path = get_package_share_directory('amr_project')
    
    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bot_description_path, 'launch', 'gazebo.launch.py')
        ])
    )
    
    # Launch SLAM (with delay to let Gazebo start)
    slam_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(bot_description_path, 'launch', 'slam.launch.py')
                ])
            )
        ]
    )
    
    return LaunchDescription([
        gazebo_launch,
        slam_launch,
    ])