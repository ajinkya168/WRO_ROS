from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import LogInfo
import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_dir = get_package_share_directory('urdf_lidar_bot')


    channel_type =  LaunchConfiguration('channel_type', default='serial')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='460800')
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')
    scan_mode = LaunchConfiguration('scan_mode', default='Standard')
    world_file = os.path.join(package_dir, 'worlds', "worlds", 'wro_2025.world')
    urdf_file = os.path.join(package_dir, 'urdf', 'lidar_bot.urdf')
    rviz_config_file = os.path.join(get_package_share_directory('urdf_lidar_bot'), 'rviz', 'lidar_bot.rviz')
    slam_params = os.path.join(get_package_share_directory("urdf_lidar_bot"), "config", "slam_config.yaml")   
    map_file = os.path.join(package_dir, "config", "wro.yaml")
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()


    # Read the SDF contents
    print("URDF File: ", urdf_file)

    print("World File: ", world_file) 
    return LaunchDescription([	
 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={"world": world_file}.items()

        ),    
      
        # Start SLAM Toolbox
	IncludeLaunchDescription(
	    PythonLaunchDescriptionSource(	
		os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")
		),
	    launch_arguments={'slam_params_file': slam_params}.items()
	), 
	

	
        DeclareLaunchArgument(
            'channel_type',
            default_value=channel_type,
            description='Specifying channel type of lidar'),
        
        DeclareLaunchArgument(
            'serial_port',
            default_value=serial_port,
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value=frame_id,
            description='Specifying frame_id of lidar'),

        DeclareLaunchArgument(
            'inverted',
            default_value=inverted,
            description='Specifying whether or not to invert scan data'),

        DeclareLaunchArgument(
            'angle_compensate',
            default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),
        DeclareLaunchArgument(
            'scan_mode',
            default_value=scan_mode,
            description='Specifying scan mode of lidar'),    
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'lidar_bot', "-x", "0", "-y", "-1", "-z", "0.5"],	
            output='screen',
            parameters=[{'use_sim_time': True}],
            condition=launch.conditions.IfCondition(LaunchConfiguration('spawn_robot', default='true')),
        ),        
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description, 'use_sim_time':True}]
        ),
        
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
        ),
        
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'channel_type':channel_type,
                         'serial_port': serial_port,
                         'serial_baudrate': serial_baudrate,
                         'frame_id': frame_id,
                         'inverted': inverted,
                         'angle_compensate': angle_compensate,
                         'scan_mode': scan_mode
                         }],
            output='screen'),
            
        Node(
            package='urdf_lidar_bot',
            executable='odom_to_base_link_tf.py',
            name='odom_to_base_link_tf',
            output='screen'
        ),
        
        # Joystick driver node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[
                # You can specify your joystick device here if needed, e.g.:
                # {'dev': '/dev/input/js0'}
            ],
        ),

        # Your teleop node subscribing to /joy and publishing /cmd_vel
        Node(
            package='urdf_lidar_bot',
            executable='joystick_teleop.py',
            name='joystick_teleop',
            output='screen',
        ),	
	
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': True}],
            output='screen'
            ) 
            ])
