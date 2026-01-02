from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Get the package directory
    bot_description_path = get_package_share_directory('amr_project')
    
    # Path to controller configuration
    controller_params_file = os.path.join(
        bot_description_path,
        'config',
        'controllers.yaml'
    )
    
    # Controller Manager Node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_params_file],
        output='screen',
    )
    
    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Diff Drive Controller Spawner
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Delay diff_drive_controller spawner after joint_state_broadcaster
    delay_diff_drive_spawner_after_joint_state = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )
    
    return LaunchDescription([
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_diff_drive_spawner_after_joint_state,
    ])