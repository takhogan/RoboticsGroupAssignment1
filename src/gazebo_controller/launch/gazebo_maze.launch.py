from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, Command
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from numpy import pi

def generate_launch_description():
    urdf_package = "en613_description"
    urdf_filename = "basic_robot.urdf.xacro"
    rviz_config_filename = "basic_robot.rviz"

    project_gz_package_name: str = "en613_gazebo"
    gz_package_name: str = "ros_gz_sim"
    gz_launch_filename: str = "gz_sim.launch.py"

    pkg_share_description = FindPackageShare(urdf_package)
    urdf_path = PathJoinSubstitution(
        [pkg_share_description, "urdf", urdf_filename]
    )
    default_rviz_path = PathJoinSubstitution(
        [pkg_share_description, "rviz", rviz_config_filename]
    )
    project_gz_path: FindPackageShare = FindPackageShare(project_gz_package_name)
    project_world_directory: PathJoinSubstitution = PathJoinSubstitution(
        [project_gz_path, "worlds"]
    )
    gz_path: FindPackageShare = FindPackageShare(gz_package_name)
    gz_launch_path: PathJoinSubstitution = PathJoinSubstitution(
        [gz_path, "launch", gz_launch_filename]
    )

    # --- 2. LAUNCH ARGUMENTS ---
    declare_rviz_config_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=default_rviz_path,
        description="Path to RViz config file",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        choices=["true", "false"],
        description="Launch RViz with the robot description",
    )
    declare_urdf_model_cmd = DeclareLaunchArgument(
        "urdf_model",
        default_value=urdf_path,
        description="Path to the URDF model file",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
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
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    urdf_model = LaunchConfiguration("urdf_model")

    # --- 3. GLOBAL PARAMETER (REQUIRED FOR SIMULATION TIME SYNC) ---
    # set_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)

    robot_description_content: ParameterValue = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' '
    ]), value_type=str)

    # --- 4. GAZEBO LAUNCH ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_launch_path),
        launch_arguments={
            'gz_args': [
                TextSubstitution(text='-r -v 4 '),
                PathJoinSubstitution([
                    project_world_directory, 'maze.world'
                ]),
            ]
        }.items(),
    )

    robot_description_content: ParameterValue = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
    ]), value_type=str)

    robot_description = {'robot_description': robot_description_content,
                         'use_sim_time': True}
    
    # --- 5. ROBOT SPAWN ---
    gz_spawn_entity_cmd: Node = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-topic", 'robot_description', '-name', 'basic_robot', '-x', '0', '-y', '0', '-z', '0.4']
    )

    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'basic_robot', 'chassis'],
        output='screen'
    )

    # --- 7. ROBOT STATE PUBLISHER (For RViz to read the robot model) ---
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description])
    
    # --- 8. PID CONTROLLER NODE ---
    pid_controller_node = Node(
        package='gazebo_controller',
        executable='pid_controller',
        name='pid_controller',
        output='screen',
        parameters=[{
            'kp': 8.0,
            'kd': 0.03,
            'max_linear_velocity': 3.0,
            'max_angular_velocity': pi,
            'position_tolerance': 1e-3}],
        condition=IfCondition(use_pid_controller)
    )
    
    # --- 8b. TELEOP NODE ---
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard', 
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e', 
        condition=IfCondition(use_teleop)
    )
    
    # --- 9. RViz2 NODE ---
    start_rviz_cmd: Node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    start_gz_bridge_cmd: Node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'qos_overrides./model/basic_robot.subscriber.reliability': 'reliable',
                'qos_overrides./tf.publisher.durability': 'transient_local',
                'qos_overrides./tf_static.publisher.durability': 'transient_local',
            }
        ],
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                   '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                   '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/model/basic_robot/pose@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                   '/model/basic_robot/pose_static@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V',
                   '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model',
                     ],
        remappings=[
            ('/model/basic_robot/pose', '/tf'),
            ('/model/basic_robot/pose_static', '/tf_static'),
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_rviz_config_cmd,
        declare_use_rviz_cmd,
        declare_urdf_model_cmd,
        use_sim_time_arg,
        use_teleop_arg, 
        use_pid_controller_arg, 
        gazebo_launch,
        robot_state_publisher,
        gz_spawn_entity_cmd,
        teleop_node,
        start_rviz_cmd,
        pid_controller_node,
        start_gz_bridge_cmd,
        static_tf_publisher
    ])
