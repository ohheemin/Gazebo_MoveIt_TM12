import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, TimerAction, AppendEnvironmentVariable, ExecuteProcess, GroupAction
from launch_ros.actions import PushRosNamespace
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    gazebo_moveit_pkg_path = get_package_share_directory("gazebo_test_bed")

    share_dir = os.path.dirname(gazebo_moveit_pkg_path)

    resource_dir = '/home/ohheemin/ros2_ws/src/gazebo_test_bed/meshes/Environment'
    dae_filename = 'Environment.dae'
    stl_filename = 'Environment.stl'
    obj_filename = 'Environment.obj'

    full_dae_path = os.path.join(resource_dir, dae_filename)
    dae_uri = f'file://{full_dae_path}'
    
    full_stl_path = os.path.join(resource_dir, stl_filename)
    stl_uri = f'file://{full_stl_path}'

    full_obj_path = os.path.join(resource_dir, obj_filename)
    obj_uri = f'file://{full_obj_path}'

    set_resource_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_dir
    )
    environment_sdf_content = f"""
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="MyEnvironment">
    <static>true</static>
    <link name="link">
      
      <visual name="visual">
        <geometry>
          <mesh>
            <uri>{dae_uri}</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
      </visual>

      <collision name="environment_complex_collision">
        <geometry>
          <mesh>
            <uri>{stl_uri}</uri>
            <scale>1 1 1</scale>
          </mesh>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.8</mu>
              <mu2>0.8</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1000000.0</kp>
              <kd>1.0</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>

    </link>
  </model>
</sdf>
"""

    block_sdf_content = """
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="target_block">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.000166667</ixx>
          <iyy>0.000166667</iyy>
          <izz>0.000166667</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.34 0.25 0.21</size>
          </box>
        </geometry>
        <material>
          <ambient>0.6 0.4 0.2 1</ambient>
          <diffuse>0.6 0.4 0.2 1</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.34 0.25 0.21</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1000000.0</kp>
              <kd>1.0</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
    </link>
  </model>
</sdf>
"""

    block_sdf_content2 = """
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="target_block2">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.000166667</ixx>
          <iyy>0.000166667</iyy>
          <izz>0.000166667</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.34 0.25 0.21</size>
          </box>
        </geometry>
        <material>
          <ambient>0.6 0.4 0.2 1</ambient>
          <diffuse>0.6 0.4 0.2 1</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.34 0.25 0.21</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1000000.0</kp>
              <kd>1.0</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
    </link>
  </model>
</sdf>
"""

    block_sdf_content3 = """
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="target_block3">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.000166667</ixx>
          <iyy>0.000166667</iyy>
          <izz>0.000166667</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.34 0.25 0.21</size>
          </box>
        </geometry>
        <material>
          <ambient>0.6 0.4 0.2 1</ambient>
          <diffuse>0.6 0.4 0.2 1</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.34 0.25 0.21</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1000000.0</kp>
              <kd>1.0</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
    </link>
  </model>
</sdf>
"""

    block_sdf_content4 = """
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="target_block4">
    <static>false</static>
    <link name="link">
      <inertial>
        <mass>0.1</mass>
        <inertia>
          <ixx>0.000166667</ixx>
          <iyy>0.000166667</iyy>
          <izz>0.000166667</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.34 0.25 0.21</size>
          </box>
        </geometry>
        <material>
          <ambient>0.6 0.4 0.2 1</ambient>
          <diffuse>0.6 0.4 0.2 1</diffuse>
          <specular>0.3 0.3 0.3 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.34 0.25 0.21</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>1.0</mu>
              <mu2>1.0</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1000000.0</kp>
              <kd>1.0</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
    </link>
  </model>
</sdf>
"""

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
    
    urdf_file_path = os.path.join(gazebo_moveit_pkg_path, "xacro", "tm12.urdf.xacro")
    mobile_urdf_file_path = os.path.join(gazebo_moveit_pkg_path, "xacro", "mobile_manipulator.urdf.xacro")
    srdf_file_path = os.path.join(gazebo_moveit_pkg_path, "srdf", "tm12.srdf")
    controllers_file_path = os.path.join(gazebo_moveit_pkg_path, "tm_config", "moveit_controllers.yaml")
    kinematics_file_path = os.path.join(gazebo_moveit_pkg_path, "tm_config", "kinematics.yaml")
    joint_limits_file_path = os.path.join(gazebo_moveit_pkg_path, "tm_config", "joint_limits.yaml")
    
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

    mobile_robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            mobile_urdf_file_path,
            " ",
            "sim_ignition:=true ",
        ]
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": "-r -v 1 empty.sdf"}.items(),
    )
    
    ros_gz_bridge = Node(   
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tm12_gripper/enable@std_msgs/msg/Empty]ignition.msgs.Empty',
            '/tm12_gripper/attach@std_msgs/msg/Empty]ignition.msgs.Empty',
            '/tm12_gripper/state@std_msgs/msg/Bool[ignition.msgs.Boolean',
        ]
    )

    send_detach_command = ExecuteProcess(
        cmd=['ign', 'topic', '-t', '/tm12_gripper/detach', 
             '-m', 'ignition.msgs.Empty', '-p', '""'],
        output='screen'
    )

    spawn_environment = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', environment_sdf_content,
            '-name', 'MyEnvironment',
            '-x', '0', '-y', '0', '-z', '0',
            '-R', '0', '-P', '0', '-Y', '3.14159'
        ],
    )

    spawn_block = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', block_sdf_content,
            '-name', 'target_block',
            '-x', '0.95',  
            '-y', '4.0',
            '-z', '3.0',  
            '-R', '0', '-P', '0', '-Y', '1.5708'
        ],
    )

    spawn_block2 = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', block_sdf_content2,
            '-name', 'target_block2',
            '-x', '0.95',
            '-y', '4.0',
            '-z', '3.0',
            '-R', '0', '-P', '0', '-Y', '1.5708'
        ],
    )

    spawn_block3 = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', block_sdf_content3,
            '-name', 'target_block3',
            '-x', '0.95',
            '-y', '4.0',
            '-z', '3.0',
            '-R', '0', '-P', '0', '-Y', '1.5708'
        ],
    )

    spawn_block4 = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', block_sdf_content4,
            '-name', 'target_block4',
            '-x', '0.95',
            '-y', '4.0',
            '-z', '3.0',
            '-R', '0', '-P', '0', '-Y', '1.5708'
        ],
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

    generate_mobile_urdf = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'xacro ' + mobile_urdf_file_path + ' sim_ignition:=true > /tmp/mobile_manipulator_generated.urdf'
        ],
        output='screen',
    )

    gz_spawn_mobile = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-file", "/tmp/mobile_manipulator_generated.urdf",
            "-name", "mobile_manipulator",
            "-allow_renaming", "true",
            "-x", "0.0", "-y", "0.0", "-z", "0.05",
        ],
    )

    mobile_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="mobile_robot_state_publisher",
        namespace="mobile_manipulator",
        output="both",
        parameters=[
            {"robot_description": ParameterValue(mobile_robot_description_content, value_type=str)},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ('/mobile_manipulator/tf', '/tf'),
            ('/mobile_manipulator/tf_static', '/tf_static'),
        ],
    )

    mobile_jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/mobile_manipulator/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )
    mobile_arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_position_example_controller", "--controller-manager", "/mobile_manipulator/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )
    mobile_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/mobile_manipulator/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    moveit_config = (
        MoveItConfigsBuilder("tm12", package_name="gazebo_test_bed")
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
        remappings=[("joint_states", "joint_states")],
        arguments=["--ros-args", "--log-level", "info"],
    )
    
    world2robot_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_world_to_robot",
        output="log",
        arguments=[
            "0.8", "4.6", "0.63",
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
    
    joint_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_position_example_controller", "--controller-manager", "/controller_manager"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )
    
    pick_place_node = Node(
        package="gazebo_test_bed",
        executable="tm12_pick_and_place",
        name="pick_and_place_cpp",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    delayed_spawn_environment = TimerAction(
        period=1.0,
        actions=[spawn_environment]
    )

    delayed_detach = TimerAction(
        period=7.0,
        actions=[send_detach_command]
    )

    delayed_spawn_block = TimerAction(
        period=5.0, 
        actions=[spawn_block]
    )

    delayed_spawn_block2 = TimerAction(
        period=55.0,
        actions=[spawn_block2]
    )

    delayed_spawn_block3 = TimerAction(
        period=110.0,  # 1분 30초
        actions=[spawn_block3]
    )

    delayed_spawn_block4 = TimerAction(
        period=170.0,  # 2분
        actions=[spawn_block4]
    )

    delayed_joint_state_broadcaster_spawner = TimerAction(
        period=12.0,
        actions=[joint_state_broadcaster_spawner]
    )
    
    delayed_joint_position_controller_spawner = TimerAction(
        period=15.0,
        actions=[joint_position_controller_spawner]
    )

    delayed_move_group_node = TimerAction(
        period=5.0,
        actions=[move_group_node]
    )

    delayed_pick_place_node = TimerAction(
        period=14.0,
        actions=[pick_place_node]
    )

    delayed_generate_mobile_urdf = TimerAction(period=1.5, actions=[generate_mobile_urdf])
    delayed_spawn_mobile = TimerAction(period=2.5, actions=[gz_spawn_mobile])
    delayed_mobile_rsp   = TimerAction(period=2.5, actions=[mobile_robot_state_publisher])
    delayed_mobile_jsb   = TimerAction(period=10.0, actions=[mobile_jsb_spawner])
    delayed_mobile_arm   = TimerAction(period=14.0, actions=[mobile_arm_spawner])
    delayed_mobile_drive = TimerAction(period=18.0, actions=[mobile_drive_spawner])
    
    return LaunchDescription([
        set_resource_path,
        use_sim_time,
        ros2_control_hardware_type,
        gz_launch,
        ros_gz_bridge,
        delayed_spawn_environment,  
        delayed_spawn_block,
        delayed_spawn_block2,
        delayed_spawn_block3,
        delayed_spawn_block4,
        gz_spawn_entity,  
        robot_state_publisher,
        world2robot_tf_node,
        delayed_detach,
        delayed_joint_state_broadcaster_spawner,
        delayed_joint_position_controller_spawner,
        delayed_move_group_node,
        delayed_pick_place_node,
        # mobile_manipulator
        delayed_generate_mobile_urdf,
        delayed_spawn_mobile,
        delayed_mobile_rsp,
        delayed_mobile_jsb,
        delayed_mobile_arm,
        delayed_mobile_drive,
    ])
