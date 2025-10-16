import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution

def generate_launch_description():

    package_name = 'gazebo_controller'

    # 1. Paths
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory(package_name)
    
    world_file_path = os.path.join(pkg_share, 'worlds', 'maze.world') 
    
    robot_sdf_file = 'diff_drive_robot.sdf'
    robot_description_content = os.path.join(
        get_package_share_directory(package_name), 
        'models', 
        'diff_drive_robot', 
        robot_sdf_file
    )
    
    with open(robot_description_content, 'r') as infp:
        robot_desc = infp.read()

    robot_sdf_path = os.path.join(pkg_share, 'models', 'diff_drive_robot', 'diff_drive_robot.sdf') 
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Start Gazebo server and client
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': '"' + world_file_path + '"',
            'use_sim_time': use_sim_time
        }.items()
    )

    # Spawn the robot entity in Gazebo (using the SDF file directly)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'diff_drive_robot', 
            '-file', '"' + robot_sdf_path + '"',
            '-x', '0.0', 
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen'
    )
    
    '''
    # Start the PID Controller node
    # Executable name matches 'pid_controller_node' from setup.py
    pid_controller_node = Node(
        package=package_name,
        executable='pid_controller_node', 
        name='pid_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    '''

    # Use the 'robot_state_publisher' to load the robot model (by passing the SDF file)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_desc,
                'use_sim_time': use_sim_time
            }
        ]
    )

    # Start RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        gazebo,
        spawn_entity,
        # pid_controller_node,
        robot_state_publisher,
        rviz_node
    ])
