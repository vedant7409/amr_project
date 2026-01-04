from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bot_description_path = get_package_share_directory('amr_project')
    bot_controller_path = get_package_share_directory('amr_controller')
    
    # Robot model argument
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(bot_description_path, 'urdf', 'bot.urdf.xacro'),
        description='Absolute path to robot urdf file'
    )

    # Process the xacro file
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), 
        value_type=str
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    # Set Gazebo resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[os.path.join(bot_description_path, 'models')]
    )

    # Launch Gazebo
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments=[
            ('gz_args', '-v 4 -r empty.sdf')
        ]
    )

    # Spawn robot in Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'bot'
        ]
    )

    # Controller configuration file
    controller_config = os.path.join(bot_controller_path, 'config', 'controllers.yaml')

    # Spawn joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--param-file', controller_config],
        output='screen'
    )

    # Spawn velocity controller
    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['simple_velocity_controller', '--param-file', controller_config],
        output='screen'
    )

    # Delay controller spawning to allow Gazebo to initialize
    delayed_joint_state_broadcaster = TimerAction(
        period=5.0,
        actions=[joint_state_broadcaster_spawner]
    )

    # Spawn velocity controller after joint state broadcaster succeeds
    delayed_velocity_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[velocity_controller_spawner]
        )
    )

    return LaunchDescription([
        model_arg,
        gazebo_resource_path,
        gazebo_launch,
        robot_state_publisher_node,
        gz_spawn_entity,
        delayed_joint_state_broadcaster,
        delayed_velocity_controller
    ])