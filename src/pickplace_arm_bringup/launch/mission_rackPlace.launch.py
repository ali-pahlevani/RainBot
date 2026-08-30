import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

from pickplace_arm_bringup.hospital_pickplace_layout import (
    COLUMNS, COLUMN_YAW, RACKS, RACK_SPAWN_Z, SPAWN, TABLE, TABLE_Z)


def generate_launch_description():
    """mission_pickPlace's run in the AWS hospital, carrying racks."""
    desc_share = get_package_share_directory('pickplace_arm_description')
    bringup_share = get_package_share_directory('pickplace_arm_bringup')
    models = os.path.join(desc_share, 'models')
    sim = {'use_sim_time': True}

    def spawn(model, name, x, y, z, yaw=0.0):
        return Node(package='ros_gz_sim', executable='create', output='screen',
                    arguments=['-world', 'aws_hospital',
                               '-file', os.path.join(models, model, 'model.sdf'),
                               '-name', name, '-x', str(x), '-y', str(y),
                               '-z', str(z), '-Y', str(yaw)])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, 'launch', 'gazebo.launch.py')))

    moveit_config = MoveItConfigsBuilder(
        'pickplace_arm', package_name='pickplace_arm_moveit_config').to_moveit_configs()
    move_group = Node(
        package='moveit_ros_move_group', executable='move_group', output='screen',
        parameters=[moveit_config.to_dict(), sim,
                    {'trajectory_execution.allowed_start_tolerance': 0.1}])

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[sim, {'yaml_filename': os.path.join(
            bringup_share, 'maps', 'aws_hospital.yaml')}])
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[os.path.join(bringup_share, 'config', 'amcl_hospital.yaml'), sim])
    localization_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[sim, {'autostart': True, 'bond_timeout': 0.0,
                          'node_names': ['map_server', 'amcl']}])
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'nav2.launch.py')))

    table = spawn('table', 'table', TABLE[0], TABLE[1], TABLE_Z, TABLE[2])
    racks = [spawn(f'rack_{c}', f'rack_{c}', xy[0], xy[1], RACK_SPAWN_Z)
             for c, xy in RACKS]
    columns = [spawn(f'apriltag_column_{i + 1}', f'apriltag_column_{i + 1}',
                     xy[0], xy[1], 0.0, COLUMN_YAW)
               for i, _h, xy in COLUMNS]

    mission = Node(
        package='pickplace_arm_bringup', executable='mission_rackPlace',
        output='screen', parameters=[sim])

    # ---- readiness gates -----------------------------------------------------
    def gate(label, timeout, *args):
        return Node(package='pickplace_arm_bringup', executable='wait_for',
                    name=f'wait_for_{label}', output='screen', parameters=[sim],
                    arguments=['--label', label, '--timeout', str(timeout), *args])

    gate_world = gate('world', 300.0, '--clock-stable', '0.5',
                      '--tf', 'odom', 'base_link')
    gate_clock = gate('clock', 300.0, '--clock-stable', '3.0',
                      '--tf', 'odom', 'base_link')
    gate_lifecycle = gate('lifecycle', 180.0,
                          '--service', '/map_server/get_state',
                          '--service', '/amcl/get_state')
    gate_amcl = gate('amcl', 300.0, '--tf', 'map', 'base_link',
                     '--topic', '/amcl_pose')
    gate_nav2 = gate('nav2', 300.0, '--action', '/navigate_to_pose',
                     '--action', '/move_action')

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        prefix=('env -u GTK_PATH -u GTK_EXE_PREFIX -u LOCPATH '
                '-u GDK_PIXBUF_MODULE_FILE -u GDK_PIXBUF_MODULEDIR '
                '-u GIO_MODULE_DIR -u GTK_IM_MODULE_FILE'),
        arguments=['-d', os.path.join(bringup_share, 'config', 'mission.rviz')],
        parameters=[moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits, sim])

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_gazebo_gui', default_value='true'),
        SetEnvironmentVariable('WORLD', 'aws_hospital.sdf'),
        SetEnvironmentVariable('SPAWN_X', str(SPAWN[0])),
        SetEnvironmentVariable('SPAWN_Y', str(SPAWN[1])),
        SetEnvironmentVariable('SPAWN_YAW', str(SPAWN[2])),
        SetEnvironmentVariable('SPAWN_DELAY', '45'),
        SetEnvironmentVariable('HEADLESS', PythonExpression(
            ["'0' if '", LaunchConfiguration('use_gazebo_gui'),
             "' == 'true' else '1'"])),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            os.path.join(bringup_share, 'config', 'fastdds_udp_only.xml')),
        gazebo,
        move_group,
        rviz,
        gate_world,
        RegisterEventHandler(OnProcessExit(
            target_action=gate_world, on_exit=[table] + columns)),
        RegisterEventHandler(OnProcessExit(target_action=table, on_exit=racks)),
        RegisterEventHandler(OnProcessExit(
            target_action=gate_world, on_exit=[gate_clock])),
        RegisterEventHandler(OnProcessExit(
            target_action=gate_clock,
            on_exit=[map_server, amcl, gate_lifecycle])),
        RegisterEventHandler(OnProcessExit(
            target_action=gate_lifecycle,
            on_exit=[localization_lifecycle, gate_amcl])),
        RegisterEventHandler(OnProcessExit(
            target_action=gate_amcl, on_exit=[nav2, gate_nav2])),
        RegisterEventHandler(OnProcessExit(
            target_action=gate_nav2, on_exit=[mission])),
    ])
