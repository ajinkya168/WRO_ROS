from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.actions import LogInfo
import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_dir = get_package_share_directory('urdf_lidar_bot')


    world_file = os.path.join(package_dir, 'worlds', "worlds", 'wro_2025_o.world')
    urdf_file = os.path.join(package_dir, 'urdf', 'lidar_bot_camera.urdf')
    rviz_config_file = os.path.join(get_package_share_directory('urdf_lidar_bot'), 'rviz', 'lidar_bot.rviz')
    map_file = os.path.join(package_dir, "config", "wro_2025.yaml")
    amcl_params_file = os.path.join(package_dir, "config", "amcl.yaml")


    nav_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="nav_manager",
        parameters=[
            {"use_sim_time": True},
            {"autostart": True},
            {"node_names": ["map_server", "amcl"]},
        ],
    )


    
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
	
	    Node(
	        package='nav2_map_server',
	        executable='map_server',
	        name='map_server',
	        output='screen',
	        parameters=[{'yaml_filename': map_file, 'use_sim_time': True}]
	    ),
	     
       nav_manager,
      
       IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('urdf_lidar_bot'), 'launch', 'localization_launch.py')
            )
        ),
        

                		        		
	   Node(
	        package = 'nav2_amcl',
	        executable = 'amcl',
	        name = 'amcl',
	        output = 'screen',
	        parameters =[{'use_sim_time':True}]   
	        
	    ),

	    
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'lidar_bot', "-x", "0", "-y", "1", "-z", "0.05", "-Y", "3.14159"],	
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),        


        ExecuteProcess(
            cmd=['ros2', 'launch', 'urdf_lidar_bot', 'navigation_launch.py', 'use_sim_time:=true', 'map_subscribe_transient_local:=true'],
            shell=True
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
            parameters=[{'use_sim_time':True}]
        ),
        
            
        Node(
            package='urdf_lidar_bot',
            executable='odom_to_base_link_tf.py',
            name='odom_to_base_link_tf',
            parameters=[{'use_sim_time':True}],
            output='screen'
        ),
        
        # Joystick driver node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'use_sim_time':True}],   # You can specify your joystick device here if needed, e.g.:# {'dev': '/dev/input/js0'}
        ),

        # Your teleop node subscribing to /joy and publishing /cmd_vel
        Node(
            package='urdf_lidar_bot',
            executable='joystick_teleop.py',
            name='joystick_teleop',
            output='screen',
            parameters=[{'use_sim_time':True}],
        ),	
	
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': True}],
            output='screen'
            ),


 
    
            ])
