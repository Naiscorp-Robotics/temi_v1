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
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
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
    # RViz configuration
    rviz_config_file = LaunchConfiguration('rvizconfig')
    declare_rviz_config = DeclareLaunchArgument(
        'rvizconfig',
        default_value=os.path.join(
            get_package_share_directory('temi_v1'),
            'rviz',
            'config.rviz'
        ),
        description='Path to the rviz config file to use'
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

    ld = LaunchDescription([
        declare_use_sim_time,
        declare_rviz_config,
        bridge_clock,
        bridge_cmd_vel,
        load_joint_state_controller,
        load_diff_drive_controller,
        relay_cmd_vel,
        rviz_node,
    ])
    return ld
    