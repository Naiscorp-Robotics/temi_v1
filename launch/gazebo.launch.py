import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Package configuration
    package_description = "temi_v1"
    package_directory = get_package_share_directory(package_description)


    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true"
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    #URDF CONFIGURATION
    urdf_file = "TEMI-V1.urdf.xacro"
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)

    if not os.path.exists(robot_desc_path):
        raise FileNotFoundError(f"URDF file not found: {robot_desc_path}")

    print(f"Loading URDF file from: {robot_desc_path}")
    robot_description_content = Command(['xacro ',robot_desc_path, ' use_sim:=true'])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        emulate_tty=True,
        parameters=[{'use_sim_time': use_sim_time}, {'robot_description': robot_description}],
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
    )
    # Gazebo Resource Path Configuration
    pkg_share = get_package_share_directory(package_description)
    gz_models_path = ":".join([
        pkg_share,
        os.path.join(pkg_share, "models"),
        os.path.join(pkg_share, "world"),
        os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    ])

    # Environment variables for Gazebo
    gz_env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH':
            ':'.join([os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
                    os.environ.get('LD_LIBRARY_PATH', default='')]),
        'IGN_GAZEBO_SYSTEM_PLUGIN_PATH':
            ':'.join([os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', default=''),
                    os.environ.get('LD_LIBRARY_PATH', default='')]),
        'IGN_GAZEBO_RESOURCE_PATH': gz_models_path,
        'IGN_GAZEBO_RENDER_ENGINE': 'ogre2',
        'IGN_GAZEBO_RENDER_ENGINE_PATH': os.environ.get('IGN_GAZEBO_RENDER_ENGINE_PATH', '')
    }

    # World Configuration
    world_file = "empty.sdf"
    world_file_path = os.path.join(package_directory, "world", world_file)

    if not os.path.exists(world_file_path):
        raise FileNotFoundError(f"World file not found: {world_file_path}")

    print(f"Loading world file from: {world_file_path}")

    declare_world_arg = DeclareLaunchArgument(
        "world",
        default_value=world_file_path,
        description="SDF World File"
    )

    # Spawn Position Arguments
    declare_spawn_x = DeclareLaunchArgument(
        "x", 
        default_value="-2.0",
        description="Model Spawn X Axis Value"
    )
    declare_spawn_y = DeclareLaunchArgument(
        "y", 
        default_value="0.0",
        description="Model Spawn Y Axis Value"
    )
    declare_spawn_z = DeclareLaunchArgument(
        "z", 
        default_value="0.5",
        description="Model Spawn Z Axis Value"
    )

    # Headless argument
    declare_headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="False",
        description="Run Gazebo in headless mode"
    )

    run_headless = LaunchConfiguration("headless")

    # Set verbosity level for Gazebo
    # Higher values (0-4) provide more detailed output for debugging
    # 0: Error, 1: Warning, 2: Info, 3: Debug, 4: Verbose
    gz_verbosity = "4"

    # Gazebo launch configuration - headless mode
    gazebo_headless = ExecuteProcess(
        condition=IfCondition(run_headless),
        cmd=["ign", "gazebo", "-r", "-v", gz_verbosity, "-s", "--headless-rendering", LaunchConfiguration("world")],
        output="screen",
        additional_env=gz_env,
        shell=False,
    )

    # Gazebo launch configuration - GUI mode
    gazebo = ExecuteProcess(
        condition=UnlessCondition(run_headless),
        cmd=["ign", "gazebo", "-r", "-v", gz_verbosity, LaunchConfiguration("world")],
        output="screen",
        additional_env=gz_env,
        shell=False,
    )

    # Robot Spawn Node
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="temi_robot_spawn",
        arguments=[
            "-name", "temi_robot",
            "-allow_renaming", "true",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
        ],
        output="screen",
    )

    # Launch Description
    ld = LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher_node,
        # Spawn position parameters
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        gz_spawn_entity,
        declare_headless_arg,
        declare_world_arg,
        gazebo,
        gazebo_headless,
    ])

    return ld
