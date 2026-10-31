import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_fishy_fish_navigation = get_package_share_directory('fishy_fish_navigation')

    gazebo_models_path, ignore_last_dir = os.path.split(pkg_fishy_fish_navigation)
    current_resource_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    os.environ["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(
        path for path in (current_resource_path, gazebo_models_path) if path
    )

    world_arg = DeclareLaunchArgument(
        'world', default_value='fish_world.sdf',
        description='Name of the Gazebo world file to load'
    )

    model_arg = DeclareLaunchArgument(
        'model', default_value='mogi_bot.urdf',
        description='Name of the URDF description to load'
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Flag to enable use_sim_time'
    )

    # Define the path to your URDF or Xacro file
    urdf_file_path = PathJoinSubstitution([
        pkg_fishy_fish_navigation,  # Replace with your package name
        "urdf",
        LaunchConfiguration('model')  # Replace with your URDF or Xacro file
    ])

    gz_bridge_params_path = os.path.join(
        get_package_share_directory('fishy_fish_navigation'),
        'config',
        'gz_bridge.yaml'
    )

    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_fishy_fish_navigation, 'launch', 'world.launch.py'),
        ),
        launch_arguments={
        'world': LaunchConfiguration('world'),
        }.items()
    )

    # Spawn the URDF model using the `/world/<world_name>/create` service
    spawn_urdf_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "mogi_bot",
            "-topic", "robot_description",
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0' 
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Node to bridge /cmd_vel and /odom
    gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    # Node to bridge /cmd_vel and /odom
    gz_image_bridge_node = Node(
        package="ros_gz_image",
        executable="image_bridge",
        arguments=[
            "/left_eye/image",
            "/right_eye/image",
        ],
        output="screen",
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time'),
             'left_eye.image.compressed.jpeg_quality': 75,
             'right_eye.image.compressed.jpeg_quality': 75},
        ],
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': ParameterValue(
                Command(['xacro', ' ', urdf_file_path]), value_type=str),
             'use_sim_time': LaunchConfiguration('use_sim_time')},
        ]
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_fishy_fish_navigation, 'config', 'ekf.yaml'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ]
    )

    # Static transforms for Gazebo sensor frames
    # Gazebo (via ros_gz_bridge) uses namespaced frame_ids like
    # 'mogi_bot/base_footprint/gpu_lidar' and 'mogi_bot/base_footprint/imu'
    # in the LaserScan and Imu messages. These frames don't exist in the TF
    # tree published by robot_state_publisher, which causes message filters
    # in RViz and slam_toolbox to drop messages and prevents mapping.
    #
    # We publish zero-offset static transforms from the logical URDF frames
    # to these Gazebo sensor frames so that TF has a complete chain:
    #   base_footprint -> base_link -> scan_link -> mogi_bot/base_footprint/gpu_lidar
    #   base_footprint -> base_link -> imu_link  -> mogi_bot/base_footprint/imu

    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_laser',
        arguments=[
            '0', '0', '0',        # x y z
            '0', '0', '0',        # roll pitch yaw
            'scan_link',          # parent frame (URDF)
            'mogi_bot/base_footprint/gpu_lidar',  # child frame (Gazebo sensor frame_id)
        ],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )

    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu',
        arguments=[
            '0', '0', '0',
            '0', '0', '0',
            'imu_link',
            'mogi_bot/base_footprint/imu',
        ],
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
    )

    obj_det_node = Node(
        package='fishy_fish_navigation',
        executable='object_detection_node.py',
        name='object_detection_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Joint State Broadcaster
    load_joint_state_controller = TimerAction(
        period=10.0,  # Delay in seconds
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager-timeout", "30",
                    "--switch-timeout", "20",
                ],
                parameters=[{'use_sim_time': True}],
                output="screen"
            )
        ]
    )

    # Bounded position controller for the articulated swimming gait
    load_fish_joint_controller = TimerAction(
        period=15.0,  # Delay in seconds
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "fish_joint_controller",
                    "--controller-manager-timeout", "30",
                    "--switch-timeout", "20",
                ],
                parameters=[{'use_sim_time': True}],
                output="screen"
            )
        ]
    )

    # Start planar velocity output immediately. Delaying this node left the
    # yaw joint uncontrolled during startup, allowing the fish to rotate.
    fish_controller = Node(
        package='fishy_fish_navigation',
        executable='publisher.py',
        name='fish_gait_controller',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    launchDescriptionObject = LaunchDescription()

    launchDescriptionObject.add_action(world_arg)
    launchDescriptionObject.add_action(model_arg)
    launchDescriptionObject.add_action(sim_time_arg)
    launchDescriptionObject.add_action(world_launch)
    launchDescriptionObject.add_action(spawn_urdf_node)
    launchDescriptionObject.add_action(gz_bridge_node)
    launchDescriptionObject.add_action(gz_image_bridge_node)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    # launchDescriptionObject.add_action(obj_det_node)
    launchDescriptionObject.add_action(fish_controller)
    launchDescriptionObject.add_action(load_joint_state_controller)
    launchDescriptionObject.add_action(load_fish_joint_controller)
    launchDescriptionObject.add_action(ekf_node)
    launchDescriptionObject.add_action(static_tf_laser)
    launchDescriptionObject.add_action(static_tf_imu)

    return launchDescriptionObject
