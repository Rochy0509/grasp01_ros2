import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    launch_args = [
        DeclareLaunchArgument("ifname", default_value="can0"),
        DeclareLaunchArgument("cycle_time", default_value="1"),
        DeclareLaunchArgument("timeout", default_value="0"),
    ]

    planning_scene_monitor_parameters = { "publish_planning_scene": True, "publish_geometry_updates": True, "publish_state_updates": True, 
                                         "publish_transforms_updates": True, "publish_robot_description":True, "publish_robot_description_semantic":True}


    # Command-line arguments
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )


    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="grasp01_hardware",
        description="ROS 2 control hardware interface type to use for the launch file -- possible values: [mock_components, isaac]",
    )

    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="grasp01_description",
            package_name="grasp01_moveit2"
        )
        .robot_description(file_path="config/grasp01_description.urdf.xacro")
        .robot_description_semantic(file_path="config/grasp01_description.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")  # <--- LOADS CONTROLLERS
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("grasp01_moveit2"), "config", rviz_base]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            planning_scene_monitor_parameters,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    planning_scene_monitor_parameters],
    )


    ros2_controllers_path = os.path.join(
        get_package_share_directory("grasp01_moveit2"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            planning_scene_monitor_parameters,
            ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["grasp01_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )


    return LaunchDescription(
        launch_args + [
            rviz_config_arg,
            ros2_control_hardware_type,
            rviz_node,
            robot_state_publisher,
            move_group_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            controller_spawner,
        ]
    )
