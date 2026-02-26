#!/usr/bin/env python3
"""
Integrated launch file for Isaac Test Bed
Combines Navigation (RViz2), TM12 MoveIt, and Goal Trigger functionalities
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(
            get_package_share_directory("isaac_test_bed"), "maps", "map.yaml"
        ),
    )

    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(
            get_package_share_directory("isaac_test_bed"), "params", "navigation_params.yaml"
        ),
    )

    nav2_bringup_launch_dir = os.path.join(get_package_share_directory("nav2_bringup"), "launch")
    rviz_config_dir = os.path.join(get_package_share_directory("isaac_test_bed"), "rviz2", "navigation.rviz")

    current_dir = os.path.dirname(os.path.realpath(__file__))
    script_name = 'Goal_trigger.py'
    script_path = os.path.join(current_dir, script_name)
    
    isaac_moveit_pkg_path = get_package_share_directory("isaac_test_bed")

    urdf_file_path = os.path.join(isaac_moveit_pkg_path, "xacro", "tm12.urdf.xacro")
    srdf_file_path = os.path.join(isaac_moveit_pkg_path, "tm_config", "tm12.srdf")
    controllers_file_path = os.path.join(isaac_moveit_pkg_path, "tm_config", "moveit_controllers.yaml")
    kinematics_file_path = os.path.join(isaac_moveit_pkg_path, "tm_config", "kinematics.yaml")
    joint_limits_file_path = os.path.join(isaac_moveit_pkg_path, "tm_config", "joint_limits.yaml")

    ros2_control_hardware_type = LaunchConfiguration(
        "ros2_control_hardware_type",
        default="isaac"
    )

    moveit_config = (
        MoveItConfigsBuilder("tm12", package_name="isaac_test_bed")
        .robot_description(
            file_path=urdf_file_path,
            mappings={
                "ros2_control_hardware_type": ros2_control_hardware_type
            },
        )
        .robot_description_semantic(file_path=srdf_file_path)
        .trajectory_execution(file_path=controllers_file_path)
        .robot_description_kinematics(file_path=kinematics_file_path)
        .joint_limits(file_path=joint_limits_file_path)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    declare_map_arg = DeclareLaunchArgument(
        "map", 
        default_value=map_dir,
        description="Full path to map file to load"
    )
    
    declare_params_arg = DeclareLaunchArgument(
        "params_file", 
        default_value=param_dir,
        description="Full path to param file to load"
    )
    
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", 
        default_value="true",
        description="Use simulation (Omniverse Isaac Sim) clock if true"
    )
    
    declare_ros2_control_hardware_type_arg = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="isaac",
        description="ROS2 control hardware interface type"
    )
    
    declare_goal_x_arg = DeclareLaunchArgument(
        "goal_x",
        default_value="10.0",
        description="Goal X coordinate for navigation"
    )
    
    declare_goal_y_arg = DeclareLaunchArgument(
        "goal_y",
        default_value="6.5",
        description="Goal Y coordinate for navigation"
    )
    
    declare_goal_yaw_arg = DeclareLaunchArgument(
        "goal_yaw",
        default_value="0.0",
        description="Goal yaw angle in radians for navigation"
    )

    nav2_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")
        ),
        launch_arguments={
            "namespace": "",
            "use_namespace": "False",
            "rviz_config": rviz_config_dir,
        }.items(),
    )
    
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch_dir, "bringup_launch.py")
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_sim_time": use_sim_time,
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
    )
    
    goal_trigger_node = ExecuteProcess(
        cmd=[
            'python3', 
            script_path,  
            '--ros-args',
            '-p', ['goal_x:=', LaunchConfiguration('goal_x')],
            '-p', ['goal_y:=', LaunchConfiguration('goal_y')],
            '-p', ['goal_yaw:=', LaunchConfiguration('goal_yaw')]
        ],
        output='screen'
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
            {"default_planning_pipeline": "ompl"},
        ],
        remappings=[("joint_states", "isaac_joint_states")],
        arguments=["--ros-args", "--log-level", "error"],
    )

    world2robot_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_world_to_robot",
        output="log",
        arguments=[
            "0.0", "0.0", "0.0",
            "0.0", "0.0", "0.0",
            "world",
            "base"
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )

    pick_place_node = Node(
        package="gazebo_test_bed",
        executable="tm12_pick_and_place",
        name="pick_and_place_cpp",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("joint_states", "isaac_joint_states")],
        arguments=["--ros-args", "--log-level", "error"],
    )


    return LaunchDescription([
        # Launch Arguments
        declare_map_arg,
        declare_params_arg,
        declare_use_sim_time_arg,
        declare_ros2_control_hardware_type_arg,
        declare_goal_x_arg,
        declare_goal_y_arg,
        declare_goal_yaw_arg,
        
        # Navigation
        nav2_rviz_launch,
        nav2_bringup_launch,
        goal_trigger_node,
        
        # TM12 MoveIt
        move_group_node,
        robot_state_publisher,
        world2robot_tf_node,
        pick_place_node,
    ])
