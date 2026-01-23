from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bot_description_path = get_package_share_directory('amr_project')
    ros_distro = os.environ.get('ROS_DISTRO')
    is_ignition = "True" if ros_distro == "humble" else "False"
    
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(bot_description_path, 'urdf', 'bot.urdf.xacro'),
        description='Absolute path to robot urdf/xacro file'
    )

    
    world_arg = DeclareLaunchArgument(
        name='world',
        default_value='empty.world',
        description='World file name (empty.world, small_house.world, small_warehouse.world)'
    )

    
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
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen',
    )

    
    existing_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    
    
    new_paths = [
        bot_description_path,
        os.path.dirname(bot_description_path),
    ]
    
    
    if existing_path:
        new_paths.insert(0, existing_path)
    
    
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join(new_paths)
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

    
    gz_spawn_entity = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-topic', 'robot_description',
                    '-name', 'bot'
                ],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
            )
        ]
    )

    
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
            ('/scan', '/scan'),
            ('/imu', '/imu'),
            ('/camera/left/image', '/camera/left/image_raw'),
            ('/camera/right/image', '/camera/right/image_raw'),
            ('/camera/depth/image', '/camera/depth/image_raw'),
        ]
    )

    
    joint_state_broadcaster_spawner = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager',
                    '/controller_manager'
                ],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                output='screen',
            )
        ]
    )
    
    
    diff_drive_controller_spawner = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'diff_drive_controller',
                    '--controller-manager',
                    '/controller_manager'
                ],
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                output='screen',
            )
        ]
    )

    
    
    return LaunchDescription([
        use_sim_time_arg,  
        model_arg,
        world_arg,
        gazebo_resource_path,
        gazebo_launch,
        robot_state_publisher_node,
        gz_spawn_entity,
        gz_bridge,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
    ])