import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration, Command, TextSubstitution
from launch.events import matches_action
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition # <--- CRITICAL FIX 1: Import IfCondition

def generate_launch_description():

    package_name = 'gazebo_controller'

    # --- 1. CONFIGURATION AND PATHS ---
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_share = get_package_share_directory(package_name)
    
    # Assuming 'worlds' folder is set up
    world_file_path = os.path.join(pkg_share, 'worlds', 'maze.world') 
    # Assuming 'rviz' folder and config file are set up
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'config.rviz')
    
    # Read the SDF content for the robot_state_publisher
    robot_sdf_file = 'diff_drive_robot.sdf'
    # Assuming the robot model is in 'models/diff_drive_robot/'
    robot_description_path = os.path.join(
        get_package_share_directory(package_name), 
        'models', 
        'diff_drive_robot', 
        robot_sdf_file
    )
    
    with open(robot_description_path, 'r') as infp:
        robot_desc = infp.read()
    
    robot_sdf_spawn_path = os.path.join(pkg_share, 'models', 'diff_drive_robot', 'diff_drive_robot.sdf')
    
    # --- 2. LAUNCH ARGUMENTS ---
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
        
    # Declare missing arguments from your command line
    use_teleop_arg = DeclareLaunchArgument(
        'use_teleop',
        default_value=TextSubstitution(text='false'),
        description='Whether to launch teleop node.')
        
    use_pid_controller_arg = DeclareLaunchArgument(
        'use_pid_controller',
        default_value=TextSubstitution(text='true'),
        description='Whether to launch PID controller node.')
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_teleop = LaunchConfiguration('use_teleop')
    use_pid_controller = LaunchConfiguration('use_pid_controller')

    # --- 3. GLOBAL PARAMETER (REQUIRED FOR SIMULATION TIME SYNC) ---
    set_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)

    # --- 4. GAZEBO LAUNCH ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file_path,
        }.items()
    )

    # --- 5. ROBOT SPAWN ---
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'diff_drive_robot', 
            '-file', robot_sdf_spawn_path,
            '-x', '0.0', 
            '-y', '0.0',
            '-z', '0.1',
        ],
        output='screen'
    )
    
    # --- 6. STATIC TRANSFORM PUBLISHER (map -> odom) ---
    static_map_odom_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_broadcaster',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'], 
        output='screen'
    )
    
    # --- 7. ROBOT STATE PUBLISHER (For RViz to read the robot model) ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_desc,
            }
        ]
    )
    
    # --- 8. PID CONTROLLER NODE ---
    pid_controller_node = Node(
        package=package_name,
        executable='pid_controller_node', 
        name='pid_controller',
        output='screen',
        condition=IfCondition(use_pid_controller) # <--- CRITICAL FIX 2: Correctly use IfCondition
    )
    
    # --- 8b. TELEOP NODE ---
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard', 
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e', 
        condition=IfCondition(use_teleop) # <--- CRITICAL FIX 3: Correctly use IfCondition
    )
    
    # --- 9. RViz2 NODE ---
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen',
    )

    # --- 10. EVENT HANDLER: Wait for robot to spawn before starting other nodes ---
    start_delayed_nodes = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                static_map_odom_publisher,
                robot_state_publisher,
                pid_controller_node,
                teleop_node,
                rviz_node
            ]
        )
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        use_teleop_arg, 
        use_pid_controller_arg, 
        set_sim_time,
        
        gazebo_launch,
        
        spawn_entity,
        
        start_delayed_nodes
    ])
