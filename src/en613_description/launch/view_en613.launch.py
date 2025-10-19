from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Define file names
    urdf_package = "en613_description"
    urdf_filename = "basic_robot.urdf.xacro"
    rviz_config_filename = "basic_robot.rviz"

    # Define paths
    pkg_share_description = FindPackageShare(urdf_package)
    urdf_path = PathJoinSubstitution(
        [pkg_share_description, "urdf", urdf_filename]
    )
    pkg_share_description = FindPackageShare(urdf_package)
    rviz_path = PathJoinSubstitution(
        [pkg_share_description, "rviz", rviz_config_filename]
    )

    rviz_config_file = LaunchConfiguration("rviz_config_file")
    use_rviz = LaunchConfiguration("use_rviz")
    urdf_model = LaunchConfiguration("urdf_model")
    jsp_gui = LaunchConfiguration("jsp_gui")

    declare_jsp_gui_cmd = DeclareLaunchArgument(
        "jsp_gui",
        default_value="true",
        choices=["true", "false"],
        description="Launch the Joint State Publisher GUI",
    )
    declare_urdf_model_cmd = DeclareLaunchArgument(
        "urdf_model",
        default_value=urdf_path,
        description="Path to the URDF model file",
    )
    declare_rviz_config_cmd = DeclareLaunchArgument(
        "rviz_config_file",
        default_value=rviz_path,
        description="Path to RViz config file",
    )
    declare_use_rviz_cmd = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        choices=["true", "false"],
        description="Launch RViz with the robot description",
    )

    robot_description_content: ParameterValue = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
        'use_gazebo:=false ',
    ]), value_type=str)

    start_robot_state_publisher_cmd: Node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}])

    start_joint_state_publisher_cmd: Node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(jsp_gui),
    )

    start_joint_state_publisher_gui_cmd: Node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(jsp_gui),
    )

    start_rviz_cmd: Node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # Create the launch description and populate with arguments
    ld = LaunchDescription()

    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_urdf_model_cmd)
    ld.add_action(declare_jsp_gui_cmd)

    # Add actions
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_cmd)
    ld.add_action(start_rviz_cmd)

    return ld