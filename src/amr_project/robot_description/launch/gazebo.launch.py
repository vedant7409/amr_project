from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bot_description_path = get_package_share_directory('amr_project')
    ros_distro = os.environ.get('ROS_DISTRO')
    is_ignition = "True" if ros_distro == "humble" else "False"
    
    # Model argument - point to xacro file
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(bot_description_path, 'urdf', 'bot_gazebo.xacro'),
        description='Absolute path to robot urdf/xacro file'
    )

    # World argument - allow selection of different worlds
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value='empty.world',
        description='World file name (empty.world, small_house.world, small_warehouse.world)'
    )

    # Process xacro to get robot description
    robot_description = ParameterValue(
        Command([
            'xacro ', 
            LaunchConfiguration('model'),
            ' ',
            'is_ignition:=', is_ignition
        ]), 
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    )

    # Set Gazebo resource paths for models and worlds
    gazebo_models_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(bot_description_path, 'models'),
            ':',
            os.path.join(bot_description_path, 'worlds')
        ]
    )

    # Build full path to world file
    world_path = os.path.join(bot_description_path, 'worlds', LaunchConfiguration('world'))

    # Launch Gazebo with selected world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments=[
            ('gz_args', ['-v 4 -r ', world_path])
        ]
    )

    # Spawn robot entity in Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'bot'
        ]
    )

    return LaunchDescription([
        model_arg,
        world_arg,
        gazebo_models_path,
        gazebo_launch,
        robot_state_publisher_node,
        gz_spawn_entity
    ])