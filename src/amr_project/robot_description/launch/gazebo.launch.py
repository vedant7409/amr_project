from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    bot_description_path = get_package_share_directory('amr_project')
    ros_distro = os.environ.get('ROS_DISTRO')
    is_ignition = "True" if ros_distro == "humble" else "False"
    
    # Model argument - point to xacro file
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(bot_description_path, 'urdf', 'bot.urdf.xacro'),
        description='Absolute path to robot urdf/xacro file'
    )

    # World argument - allow selection of different worlds
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value='small_house.world',
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

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # Set Gazebo resource paths for models and worlds
    package_allowance_dir = os.path.abspath(os.path.join(bot_description_path, '..'))

    gazebo_models_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            package_allowance_dir,
            ':',
            os.path.join(bot_description_path, 'models'),
            ':',
            os.path.join(bot_description_path, 'worlds')
        ]
    )

    # Build full path to world file
    world_path = PathJoinSubstitution([
    bot_description_path,
    'worlds',
    LaunchConfiguration('world')
    ])

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

    # Bridge for sensor topics (LiDAR, IMU, Clock)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
        remappings=[
            ('/scan', '/scan'),
            ('/imu', '/imu'),
        ]
    )

    # Joint State Broadcaster Spawner (with delay)
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,  # Wait 3 seconds for Gazebo to fully load
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager',
                    '/controller_manager'
                ],
                output='screen',
            )
        ]
    )
    
    # Diff Drive Controller Spawner (with longer delay)
    diff_drive_controller_spawner = TimerAction(
        period=5.0,  # Wait 5 seconds
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'diff_drive_controller',
                    '--controller-manager',
                    '/controller_manager'
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        model_arg,
        world_arg,
        gazebo_models_path,
        gazebo_launch,
        robot_state_publisher_node,
        gz_spawn_entity,
        gz_bridge,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ])