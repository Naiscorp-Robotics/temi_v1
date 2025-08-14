from launch import LaunchDescription  
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node  
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory




def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    # SLAM Toolbox params file
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config_file = LaunchConfiguration('rvizconfig')

    # Nav2 params file
    nav_params_file = LaunchConfiguration('nav_params_file')
    declare_nav_params_file = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(get_package_share_directory('temi_v1'), 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )

   
    # Load controller configuration first
    controller_params_file = os.path.join(
        get_package_share_directory('temi_v1'),
        'config',
        'diff_drive_controller.yaml'
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_params_file, {'use_sim_time': use_sim_time}],
        output="screen",
    )

    # Load controller
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_diff_drive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diffbot_base_controller'],
        output='screen'
    )
    # Relay cmd_vel
    relay_cmd_vel = Node(
        name="relay_cmd_vel",
        package="topic_tools",
        executable="relay",
        parameters=[
            {
                "input_topic": "/cmd_vel",
                "output_topic": "/diffbot_base_controller/cmd_vel_unstamped",
                "use_sim_time": use_sim_time
            }
        ],
        output="screen",
    )
    # Connection established
    bridge_clock = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )
    bridge_cmd_vel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/diffbot_base_controller/cmd_vel_unstamped@geometry_msgs/msg/Twist]ignition.msgs.Twist'],
        output='screen'
    )
    bridge_lidar = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan'],
        output='screen'
    )
    
    # Bridge for odometry data from Gazebo
    bridge_odom = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
        output='screen'
    )
    
    # Bridge for joint states from Gazebo
    bridge_joint_states = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model'],
        output='screen'
    )
    # RViz node
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    # SLAM Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}]
    )
    # Nav2
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_launch_dir = os.path.join(nav2_bringup_dir, 'launch')
    
    # Sử dụng bringup_launch.py thay vì navigation_launch.py
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav_params_file,
            'map': '',  # Không cần load map trước, SLAM sẽ tạo map
            'use_composition': 'False',
            'autostart': 'True'
        }.items()
    )

    ld = LaunchDescription([
        # Launch arguments first
        declare_use_sim_time,
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(get_package_share_directory('temi_v1'), 'config', 'slam_toolbox_params.yaml'),
            description='Full path to the SLAM parameters file'
        ),
        DeclareLaunchArgument(
            'rvizconfig',
            default_value=os.path.join(get_package_share_directory('temi_v1'), 'rviz', 'navigation_with_camera.rviz'),
            description='Path to the rviz config file to use'
        ),
        declare_nav_params_file,
        
        # Core system bridges first
        bridge_clock,
        bridge_joint_states,
        bridge_odom,
        
        # Control system
        control_node,
        load_joint_state_controller,
        load_diff_drive_controller,
        
        # Command velocity handling
        relay_cmd_vel,
        bridge_cmd_vel,
        
        # Sensors
        bridge_lidar,
        
        # SLAM and Navigation (after all sensors and control are ready)
        slam_toolbox_node,
        nav2_launch,
        
        # Visualization (last)
        rviz_node,
    ])
    return ld
    