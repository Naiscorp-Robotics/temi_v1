import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # Package configuration
    package_description = "temi_v1"
    package_directory = get_package_share_directory(package_description)
    

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # Set to false for real hardware
        description='Whether to use simulation time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # URDF Configuration with use_sim=false for real hardware
    urdf_file = 'TEMI-V1.urdf.xacro'
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)

    if not os.path.exists(robot_desc_path):
        raise FileNotFoundError(f"URDF/XACRO file not found: {robot_desc_path}")

    print(f"Loading URDF/XACRO from: {robot_desc_path}")
    robot_description_content = Command(['xacro ', robot_desc_path, ' use_sim:=false'])
    robot_description = ParameterValue(robot_description_content, value_type=str)
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output="screen",
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Controller Manager Node with hardware params 
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description': robot_description},
            os.path.join(package_directory, "config", "hardware_params.yaml"),
            {'use_sim_time': use_sim_time}
        ],
        output="screen",
        arguments=['--ros-args', '--log-level', 'info']
    )

    # Joint State Broadcaster - required for tf
    # Only spawn after controller_manager is started
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", "info"],
        output="screen",
    )

    # Diff Drive Controller
    # Only spawn after controller_manager is started
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager", "--ros-args", "--log-level", "info"],
        output="screen",
    )

    # Create event handlers to spawn controllers only after controller_manager has started
    joint_state_broadcaster_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[joint_state_broadcaster_spawner]
        )
    )

    diff_drive_controller_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager_node,
            on_start=[diff_drive_controller_spawner]
        )
    )

    

    
    
    # RViz node for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig'), '--ros-args', '--log-level', 'info']
    )
    
    declare_rvizconfig = DeclareLaunchArgument(
        'rvizconfig',
        default_value=os.path.join(
            get_package_share_directory('temi_v1'),
            'rviz',
            'navigation_with_camera.rviz'),
    )
    
    
    
    # Command relayer for cmd_vel
    # Only create relay after the diff drive controller is spawned
    relay_cmd_vel = Node(
        name="relay_cmd_vel",
        package="topic_tools",
        executable="relay",
        parameters=[
            {
                "input_topic": "/cmd_vel",
                "output_topic": "/diffbot_base_controller/cmd_vel_unstamped",
                "use_sim_time": use_sim_time,
                "lazy": False,  # Don't wait for subscribers before operating
                "queue_size": 5,  # Increase queue size for better angular velocity handling
                "tcp_nodelay": True  # Reduce latency for real-time control
            }
        ],
        output="screen",
        arguments=['--ros-args', '--log-level', 'warn']  # Reduce logs to increase speed
    )

    # Event to start relay only after diff drive controller is loaded
    relay_cmd_vel_event = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=diff_drive_controller_spawner,
            on_start=[relay_cmd_vel]
        )
    )

    
    
    
    
    

    

    

    # Combine all launch components
    ld = LaunchDescription([
        declare_use_sim_time,
        # declare_ekf_params_file,
        declare_rvizconfig,
        # declare_slam_params_file,
        # declare_sllidar_params_file,
        # declare_nav_params_file,
        # # Nodes
        robot_state_publisher_node,
        controller_manager_node,
        # sllidar_node,
        # Replaced direct spawner nodes with event handlers
        joint_state_broadcaster_event,
        diff_drive_controller_event,
        relay_cmd_vel_event,
        # ekf_node,
        # slam_toolbox_node,
        # Nav2 launch
        # nav2_launch,
        # joy_teleop_launch,
        rviz_node,
        # Orbbec camera components
        # gemini2_camera_launch,
        # camera_tf_node,
        # gemini2_tf_node,
    ])

    return ld 