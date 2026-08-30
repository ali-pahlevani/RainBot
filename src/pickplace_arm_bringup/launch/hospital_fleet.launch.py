import os
import shlex
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
    IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder

from pickplace_arm_bringup.fleet_layout import (
    ARM_ROBOTS, FORMATION_CENTRE, NAV_ROBOTS, ROBOTS, SPAWN_Z, WORLD_ENTITY,
)
from pickplace_arm_bringup.fleet_rviz import fleet_rviz_config
from pickplace_arm_bringup.ns_params import diff_drive_frames, namespaced_params

# --- bringing it up faster while developing ----------------------------------

SPAWN_SETTLE = int(os.environ.get(
    'FLEET_SPAWN_SETTLE', 0 if os.environ.get('HEADLESS') == '1' else 20))
SPAWN_STAGGER = int(os.environ.get('FLEET_SPAWN_STAGGER', 6))

DDS_PROFILE = 'fastdds_udp_only.xml'

NAV_STAGGER = int(os.environ.get('FLEET_NAV_STAGGER', 12))
NAV_SETTLE = int(os.environ.get('FLEET_NAV_SETTLE', 0))

ARM_STAGGER = int(os.environ.get('FLEET_ARM_STAGGER', 15))
NAV_NODES = ['controller_server', 'smoother_server', 'planner_server',
             'behavior_server', 'bt_navigator']


def _rviz_env_prefix(bringup_share):
    """Run rviz2 in a whitelisted environment."""
    keep = ('HOME', 'USER', 'DISPLAY', 'XAUTHORITY', 'PATH', 'LD_LIBRARY_PATH',
            'AMENT_PREFIX_PATH', 'PYTHONPATH', 'ROS_DISTRO', 'ROS_VERSION',
            'ROS_PYTHON_VERSION', 'RMW_IMPLEMENTATION', 'ROS_DOMAIN_ID',
            'ROS_LOCALHOST_ONLY', 'ROS_AUTOMATIC_DISCOVERY_RANGE',
            'XDG_RUNTIME_DIR')
    pairs = [f'{k}={os.environ[k]}' for k in keep if os.environ.get(k)]
    if not os.environ.get('XAUTHORITY'):
        pairs.append('XAUTHORITY=' + os.path.expanduser('~/.Xauthority'))
    pairs.append('FASTRTPS_DEFAULT_PROFILES_FILE='
                 + os.path.join(bringup_share, 'config', DDS_PROFILE))
    return 'env -i ' + ' '.join(shlex.quote(pair) for pair in pairs)


def generate_launch_description():
    """Three robots up together in the AWS hospital, in front of reception."""
    desc_share = get_package_share_directory('pickplace_arm_description')
    bringup_share = get_package_share_directory('pickplace_arm_bringup')
    xacro_file = os.path.join(desc_share, 'urdf', 'pickplace_arm.urdf.xacro')
    world_file = os.path.join(desc_share, 'worlds', 'aws_hospital.sdf')
    map_yaml = os.path.join(bringup_share, 'maps', 'aws_hospital.yaml')
    nav2_yaml = os.path.join(bringup_share, 'config', 'nav2_params.yaml')
    amcl_yaml = os.path.join(bringup_share, 'config', 'amcl_hospital.yaml')
    sim = {'use_sim_time': True}

    gz_flags = '-s -r ' if os.environ.get('HEADLESS') == '1' else '-r '
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'),
                         'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_flags + world_file,
                          'gz_version': '8'}.items(),
    )


    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[sim, {'yaml_filename': map_yaml}])
    map_lifecycle = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map', output='screen',
        parameters=[sim, {'autostart': True, 'node_names': ['map_server'],
                          'bond_timeout': 0.0}])

    map_pump = Node(
        package='pickplace_arm_bringup', executable='map_pump',
        name='map_pump', output='screen', parameters=[sim],
        arguments=['--period', '2.0', '--duration', '1800'])

    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']
    for ns, _, _, _ in ROBOTS:
        bridge_args += [
            f'/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/{ns}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ]
        if ns in ARM_ROBOTS:
            bridge_args += [
                f'/{ns}/front_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                f'/{ns}/front_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                f'/{ns}/front_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            ]
            for colour in ('red', 'green', 'blue'):
                bridge_args += [
                    f'/{ns}/rack_{colour}/attach@std_msgs/msg/Empty]gz.msgs.Empty',
                    f'/{ns}/rack_{colour}/detach@std_msgs/msg/Empty]gz.msgs.Empty',
                ]
    bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                  arguments=bridge_args, output='screen')

    last_ns = ROBOTS[-1][0]
    fleet_gate = Node(
        package='pickplace_arm_bringup', executable='wait_for',
        name='wait_for_fleet', output='screen', parameters=[sim],
        arguments=['--label', 'fleet', '--timeout', '900',
                   '--clock-stable', '0.5',
                   '--topic', f'/{last_ns}/diff_drive_controller/odom'])

    # Filled in the loop below, started behind arm_gate at the end.
    arm_actions = []

    urdf_dir = tempfile.mkdtemp(prefix='fleet_urdf_')
    urdf_files = {}
    for ns, *_ in ROBOTS:
        urdf = subprocess.check_output(
            ['xacro', xacro_file, 'use_gazebo:=true', f'robot_ns:={ns}',
             'wrist_camera:=false'],
            text=True)
        path = os.path.join(urdf_dir, f'{ns}.urdf')
        with open(path, 'w') as fh:
            fh.write(urdf)
        urdf_files[ns] = path

    actions = []
    for idx, (ns, x, y, yaw) in enumerate(ROBOTS):
        robot_description = {
            'robot_description': ParameterValue(
                Command(['xacro ', xacro_file,
                         ' use_gazebo:=true', f' robot_ns:={ns}',
                         ' wrist_camera:=false']),
                value_type=str)
        }

        actions.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=ns, output='screen',
            parameters=[robot_description, sim, {'frame_prefix': f'{ns}/'}],
        ))

        spawn = ExecuteProcess(
            cmd=['bash', '-c',
                 f'until gz service -l 2>/dev/null | '
                 f'grep -q "^/world/{WORLD_ENTITY}/create$"; do sleep 2; done; '
                 f'sleep {SPAWN_SETTLE + idx * SPAWN_STAGGER}; '
                 f'exec ros2 run ros_gz_sim create '
                 f'-world {WORLD_ENTITY} '
                 f'-file {urdf_files[ns]} -name {ns} '
                 f'-x {x} -y {y} -z {SPAWN_Z} -Y {yaw}'],
            output='screen',
        )
        actions.append(spawn)

        def spawner(name, extra=None, _ns=ns):
            args = [name, '--controller-manager', f'/{_ns}/controller_manager',
                    '--controller-manager-timeout', '120']
            if extra:
                args += ['--param-file', extra]
            return Node(package='controller_manager', executable='spawner',
                        arguments=args, output='screen')

        jsb = spawner('joint_state_broadcaster')
        arm = spawner('arm_controller')
        grip = spawner('gripper_controller')
        diff = spawner('diff_drive_controller', diff_drive_frames(ns))
        actions += [
            RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
            RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm])),
            RegisterEventHandler(OnProcessExit(target_action=arm, on_exit=[grip])),
            RegisterEventHandler(OnProcessExit(target_action=grip, on_exit=[diff])),
        ]

        nav_actions = []
        if ns in NAV_ROBOTS:
            nav_actions.append(Node(
                package='robot_localization', executable='ekf_node',
                name='ekf_filter_node', namespace=ns, output='screen',
                parameters=[namespaced_params(
                    os.path.join(desc_share, 'config', 'ekf.yaml'), ns), sim],
            ))

            amcl_params = namespaced_params(
                amcl_yaml, ns,
                overrides={'amcl': {'initial_pose.x': x,
                                    'initial_pose.y': y,
                                    'initial_pose.yaw': yaw}})
            nav_actions.append(Node(
                package='nav2_amcl', executable='amcl', name='amcl',
                namespace=ns, output='screen',
                parameters=[amcl_params, sim],
                remappings=[('map', '/map')]))

            nav_params = namespaced_params(nav2_yaml, ns)
            for pkg, exe in (('nav2_controller', 'controller_server'),
                             ('nav2_smoother', 'smoother_server'),
                             ('nav2_planner', 'planner_server'),
                             ('nav2_behaviors', 'behavior_server'),
                             ('nav2_bt_navigator', 'bt_navigator')):
                remaps = [('map', '/map')]
                if exe == 'controller_server':
                    remaps.append(
                        ('cmd_vel',
                         f'/{ns}/diff_drive_controller/cmd_vel_unstamped'))
                nav_actions.append(Node(
                    package=pkg, executable=exe, name=exe, namespace=ns,
                    output='screen', parameters=[nav_params, sim],
                    remappings=remaps))

            nav_order = list(NAV_ROBOTS)
            prev_ns = (nav_order[nav_order.index(ns) - 1]
                       if nav_order.index(ns) > 0 else None)
            gate_args = ['--label', f'{ns} nav services', '--timeout', '300'] \
                + [arg for n in ['amcl'] + NAV_NODES
                   for arg in ('--service', f'/{ns}/{n}/change_state')]
            if prev_ns is not None:
                gate_args += ['--action', f'/{prev_ns}/navigate_to_pose']
            lifecycle_gate = Node(
                package='pickplace_arm_bringup', executable='wait_for',
                name='wait_for_nav_services', namespace=ns, output='screen',
                parameters=[sim], arguments=gate_args)

            managed = ['amcl'] + NAV_NODES
            nav_actions.append(RegisterEventHandler(
                OnProcessExit(target_action=lifecycle_gate, on_exit=[
                    Node(
                        package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        name='lifecycle_manager_navigation', namespace=ns,
                        output='screen',
                        parameters=[sim, {'autostart': False,
                                          'bond_timeout': 0.0,
                                          'node_names': managed}]),
                    Node(
                        package='pickplace_arm_bringup', executable='nav_bringup',
                        name='nav_bringup', namespace=ns, output='screen',
                        parameters=[sim],
                        arguments=['--namespace', ns]
                        + [a for n in managed for a in ('--node', n)]),
                ])))
            nav_actions.append(lifecycle_gate)
            actions.append(RegisterEventHandler(
                OnProcessExit(target_action=fleet_gate, on_exit=[TimerAction(
                    period=float(NAV_SETTLE + idx * NAV_STAGGER),
                    actions=nav_actions)])))

        if ns in ARM_ROBOTS:
            moveit_config = (
                MoveItConfigsBuilder('pickplace_arm',
                                     package_name='pickplace_arm_moveit_config')
                .robot_description(
                    mappings={'use_gazebo': 'true', 'robot_ns': ns})
                .to_moveit_configs())
            arm_actions.append(Node(
                package='moveit_ros_move_group', executable='move_group',
                namespace=ns, output='screen',
                parameters=[moveit_config.to_dict(), sim,
                            {'trajectory_execution.allowed_start_tolerance': 0.1}]))

    arm_gate = Node(
        package='pickplace_arm_bringup', executable='wait_for',
        name='wait_for_arms', output='screen', parameters=[sim],
        arguments=['--label', 'arms', '--timeout', '900']
        + [a for n in NAV_ROBOTS for a in ('--topic', f'/{n}/nav_ready')])

    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        prefix=_rviz_env_prefix(bringup_share),
        arguments=['-d', fleet_rviz_config(
            ROBOTS, arm_robots=ARM_ROBOTS, nav_robots=NAV_ROBOTS,
            view_centre=FORMATION_CENTRE)],
        parameters=[sim])

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        *[AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', p) for p in
          (os.path.dirname(desc_share),
           os.path.join(desc_share, 'models'),
           os.path.join(desc_share, 'aws_hospital_models'))],
        SetEnvironmentVariable('GZ_FUEL_CACHE_PATH',
                               os.path.join(desc_share, 'fuel_cache')),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            os.path.join(bringup_share, 'config', DDS_PROFILE)),
        gazebo,
        bridge,
        map_server,
        map_lifecycle,
        map_pump,
        fleet_gate,
        *actions,
        arm_gate,
        RegisterEventHandler(OnProcessExit(
            target_action=arm_gate,
            on_exit=[TimerAction(period=float(i * ARM_STAGGER), actions=[a])
                     for i, a in enumerate(arm_actions)] + [rviz])),
    ])
