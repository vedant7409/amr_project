from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    bot_description_path = get_package_share_directory('amr_project')
    ros_distro = os.environ.get('ROS_DISTRO')
    is_ignition = "True" if ros_distro == "humble" else "False"
    
    # 1. Declare Global Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(bot_description_path, 'urdf', 'bot.urdf.xacro'),
        description='Absolute path to robot urdf/xacro file'
    )

    world_arg = DeclareLaunchArgument(
        name='world',
        default_value='small_house.world',
        description='World file name'
    )

    # 2. Process xacro
    robot_description = ParameterValue(
        Command([
            'xacro ', 
            LaunchConfiguration('model'),
            ' ',
            'is_ignition:=', is_ignition
        ]), 
        value_type=str
    )

    # 3. Robot State Publisher (Synchronized with Sim Time)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True # Crucial for TF sync
        }],
        output='screen',
    )

    # 4. Gazebo Environment Setup
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

    world_path = PathJoinSubstitution([
        bot_description_path,
        'worlds',
        LaunchConfiguration('world')
    ])

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments=[
            ('gz_args', ['-v 4 -r ', world_path])
        ]
    )

    # 5. Spawn Entity
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'bot',
            '-allow_renaming', 'true'
        ]
    )

    
    # 6. Gazebo-ROS Bridge
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/left/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/right/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/depth/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/camera/left/image', '/camera/left/image_raw'),
            ('/camera/right/image', '/camera/right/image_raw'),
            ('/camera/depth/image', '/camera/depth/image_raw'),
        ]
    )

    # 7. Controller Spawners 
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                parameters=[{'use_sim_time': True}],
                output='screen',
            )
        ]
    )
    
    diff_drive_controller_spawner = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['diff_drive_controller'],
                parameters=[{'use_sim_time': True}],
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