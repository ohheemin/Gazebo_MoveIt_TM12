import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    isaac_moveit_pkg_path = get_package_share_directory("tm12_gazebo_moveit")
    # isaac_moveit_pkg_path = ~/gazebo_ws/install/tm12_gazebo_moveit/share/tm12_gazebo_moveit

    share_dir = os.path.dirname(isaac_moveit_pkg_path)  # share_dir = ~/gazebo_ws/install/tm12_gazebo_moveit/share

    current_gz_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    if current_gz_path:
        os.environ['GZ_SIM_RESOURCE_PATH'] = f"{current_gz_path}:{share_dir}"
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = share_dir
    
    current_gazebo_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if current_gazebo_path:
        os.environ['GAZEBO_MODEL_PATH'] = f"{current_gazebo_path}:{share_dir}"
    else:
        os.environ['GAZEBO_MODEL_PATH'] = share_dir
    
    urdf_file_path = os.path.join(isaac_moveit_pkg_path, "xacro", "tm12.urdf.xacro")
    srdf_file_path = os.path.join(isaac_moveit_pkg_path, "tm_config", "tm12.srdf")
    controllers_file_path = os.path.join(isaac_moveit_pkg_path, "tm_config", "moveit_controllers.yaml")
    kinematics_file_path = os.path.join(isaac_moveit_pkg_path, "tm_config", "kinematics.yaml")
    joint_limits_file_path = os.path.join(isaac_moveit_pkg_path, "tm_config", "joint_limits.yaml")
    
    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation clock if true"
    )
    
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="isaac",
        description="ROS2 control hardware interface type"
    )
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            urdf_file_path,
            " ",
            "hand:=true ",
            "sim_ignition:=true ",
            "simulation_controllers:=",
            controllers_file_path,
        ]
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": "-r -v 1 empty.sdf"}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "tm12",
            "-allow_renaming",
            "true",
        ],
    )
    
    moveit_config = (
        MoveItConfigsBuilder("tm12", package_name="tm12_gazebo_moveit")
        .robot_description(
            file_path=urdf_file_path,
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path=srdf_file_path)
        .trajectory_execution(file_path=controllers_file_path)
        .robot_description_kinematics(file_path=kinematics_file_path)
        .joint_limits(file_path=joint_limits_file_path)
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"default_planning_pipeline": "ompl"},
        ],
        remappings=[("joint_states", "isaac_joint_states")],
        arguments=["--ros-args", "--log-level", "info"],
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
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )
    
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )
    
    # joint_position_example_controller spawner (tmr_arm_controller 대신 사용)
    joint_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_position_example_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )
    
    pick_place_node = Node(
        package="tm12_gazebo_moveit",
        executable="tm12_pick_and_place",
        name="pick_and_place_cpp",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ]
    )
    
    # Gazebo가 로드된 후 약간의 지연을 주고 controller를 spawn
    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=3.0,
        actions=[joint_state_broadcaster_spawner]
    )
    
    delayed_joint_position_controller_spawner = TimerAction(
        period=4.0,
        actions=[joint_position_controller_spawner]
    )
    
    # move_group_node를 5초 후에 실행
    delayed_move_group_node = TimerAction(
        period=5.0,
        actions=[move_group_node]
    )
    
    # pick_place_node를 7초 후에 실행 (move_group이 준비된 후)
    delayed_pick_place_node = TimerAction(
        period=7.0,
        actions=[pick_place_node]
    )
    
    return LaunchDescription([
        use_sim_time,
        ros2_control_hardware_type,
        gz_launch,
        gz_spawn_entity,
        robot_state_publisher,
        world2robot_tf_node,
        delayed_joint_state_broadcaster_spawner,
        delayed_joint_position_controller_spawner,
        delayed_move_group_node,
        delayed_pick_place_node
    ])
