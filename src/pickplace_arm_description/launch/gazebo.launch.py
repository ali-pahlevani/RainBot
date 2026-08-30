import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, ExecuteProcess, IncludeLaunchDescription,
    RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_description = get_package_share_directory('pickplace_arm_description')

    gz_resource_paths = [
        os.path.dirname(pkg_description),
        os.path.join(pkg_description, 'models'),
        os.path.join(pkg_description, 'aws_hospital_models'),
    ]

    fuel_cache_path = os.path.join(pkg_description, 'fuel_cache')

    xacro_file = os.path.join(pkg_description, 'urdf', 'pickplace_arm.urdf.xacro')
    world_name = os.environ.get('WORLD', 'tugbot_warehouse.sdf')
    world_file = os.path.join(pkg_description, 'worlds', world_name)
    spawn_x = os.environ.get('SPAWN_X', '0.0')
    spawn_y = os.environ.get('SPAWN_Y', '0.0')
    spawn_yaw = os.environ.get('SPAWN_YAW', '0.0')
    spawn_delay = os.environ.get('SPAWN_DELAY', '0')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file, ' use_gazebo:=true']), value_type=str
        )
    }

    gz_flags = '-s -r ' if os.environ.get('HEADLESS') == '1' else '-r '
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        # gz_version 8 = Gazebo Harmonic
        launch_arguments={
            'gz_args': gz_flags + world_file,
            'gz_version': '8',
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    with open(world_file) as fh:
        world_match = re.search(r"<world\s+name=['\"]([^'\"]+)['\"]", fh.read())
    world_entity_name = world_match.group(1) if world_match else 'default'

    spawn_entity = ExecuteProcess(
        cmd=['bash', '-c',
             f'until gz service -l 2>/dev/null | '
             f'grep -q "^/world/{world_entity_name}/create$"; do sleep 2; done; '
             f'sleep {spawn_delay}; '
             f'exec ros2 run ros_gz_sim create '
             f'-world {world_entity_name} '
             f'-topic /robot_description -name pickplace_arm '
             f'-x {spawn_x} -y {spawn_y} -z 0.14 -Y {spawn_yaw}'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager',
                   '--controller-manager-timeout', '60'],
        output='screen',
    )

    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delayed_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    delayed_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    delayed_diff_drive_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gripper_controller_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Simulation clock -> ROS, so use_sim_time nodes get a time source
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Front base-mounted RGB-D camera (box detection while driving)
            '/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/front_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/box_red/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/box_red/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/box_green/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/box_green/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/box_blue/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/box_blue/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/rack_red/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/rack_red/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/rack_green/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/rack_green/detach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/rack_blue/attach@std_msgs/msg/Empty]gz.msgs.Empty',
            '/rack_blue/detach@std_msgs/msg/Empty]gz.msgs.Empty',
        ],
        output='screen',
    )

    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(pkg_description, 'config', 'ekf.yaml'),
            {'use_sim_time': True},
        ],
    )

    set_gz_resource_path = [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', p) for p in gz_resource_paths
    ]
    set_gz_fuel_cache_path = SetEnvironmentVariable('GZ_FUEL_CACHE_PATH', fuel_cache_path)

    return LaunchDescription([
        *set_gz_resource_path,
        set_gz_fuel_cache_path,
        gazebo,
        robot_state_publisher,
        spawn_entity,
        delayed_joint_state_broadcaster,
        delayed_arm_controller,
        delayed_gripper_controller,
        delayed_diff_drive_controller,
        bridge,
        ekf,
    ])
