import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction, GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap


def generate_launch_description():

    gazebo_test_bed_dir = get_package_share_directory("gazebo_test_bed")
    nav2_bringup_dir    = get_package_share_directory("nav2_bringup")

    current_dir = os.path.dirname(os.path.realpath(__file__))
    script_path = os.path.join(current_dir, 'Goal_trigger.py')

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    map_dir = LaunchConfiguration(
        "map",
        default=os.path.join(gazebo_test_bed_dir, "maps", "map.yaml"),
    )
    param_dir = LaunchConfiguration(
        "params_file",
        default=os.path.join(gazebo_test_bed_dir, "params", "navigation_params.yaml"),
    )

    nav2_bringup_launch_dir = os.path.join(nav2_bringup_dir, "launch")
    rviz_config_dir         = os.path.join(gazebo_test_bed_dir, "rviz2", "navigation.rviz")

    declare_map_arg = DeclareLaunchArgument(
        "map", default_value=map_dir, description="Full path to map yaml"
    )
    declare_params_arg = DeclareLaunchArgument(
        "params_file", default_value=param_dir, description="Full path to navigation params yaml"
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock if true"
    )

    amr_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='amr_ros_gz_bridge',
        output='screen',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/mobile_manipulator/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ]
    )

    scan_frame_relay = Node(
        package='topic_tools',
        executable='relay_field',
        name='scan_frame_relay',
        output='screen',
        arguments=[
            '/mobile_manipulator/scan',   
            '/scan',                    
            'sensor_msgs/msg/LaserScan',
            '{header: {stamp: m.header.stamp, frame_id: "base_scan"}, '
            'angle_min: m.angle_min, angle_max: m.angle_max, '
            'angle_increment: m.angle_increment, time_increment: m.time_increment, '
            'scan_time: m.scan_time, range_min: m.range_min, range_max: m.range_max, '
            'ranges: m.ranges, intensities: m.intensities}',
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch_dir, "bringup_launch.py")
        ),
        launch_arguments={
            "map":          LaunchConfiguration("map"),
            "use_sim_time": use_sim_time,
            "params_file":  LaunchConfiguration("params_file"),
            "autostart":    "true",
        }.items(),
    )

    nav2_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_launch_dir, "rviz_launch.py")
        ),
        launch_arguments={
            "namespace":     "",
            "use_namespace": "False",
            "rviz_config":   rviz_config_dir,
        }.items(),
    )

    start_trigger_node = ExecuteProcess(
        cmd=[
            'python3', script_path,
            '--ros-args',
            '-p', 'goal_x:=10.0',
            '-p', 'goal_y:=6.5',
            '-p', 'goal_yaw:=0.0',
        ],
        output='screen'
    )

    nav2_group = GroupAction([
        SetRemap(src='/cmd_vel', dst='/mobile_manipulator/diff_drive_controller/cmd_vel_unstamped'),
        nav2_bringup_launch,
    ])

    delayed_scan_relay = TimerAction(period=3.0,  actions=[scan_frame_relay])
    delayed_nav2       = TimerAction(period=12.0, actions=[nav2_group])
    delayed_rviz       = TimerAction(period=15.0, actions=[nav2_rviz_launch])
    delayed_goal       = TimerAction(period=20.0, actions=[start_trigger_node])

    return LaunchDescription([
        declare_map_arg,
        declare_params_arg,
        declare_use_sim_time_arg,
        amr_ros_gz_bridge,   
        delayed_scan_relay, 
        delayed_nav2,     
        delayed_rviz,  
        delayed_goal,      
    ])
