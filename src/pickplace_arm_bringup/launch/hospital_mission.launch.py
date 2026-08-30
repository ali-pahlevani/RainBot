import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument,
                            IncludeLaunchDescription, RegisterEventHandler,
                            TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from pickplace_arm_bringup.fleet_layout import ARM_ROBOTS, ROBOTS, WORLD_ENTITY
from pickplace_arm_bringup.rack_table_layout import (
    RACK_SPAWN_Z, STATIC_TABLES, TABLE_SPAWN_Z, collection_points,
    delivery_table_pose,
)


def generate_launch_description():
    """The fleet plus the props it works with: four tables and three racks."""
    bringup_share = get_package_share_directory('pickplace_arm_bringup')
    desc_share = get_package_share_directory('pickplace_arm_description')
    models = os.path.join(desc_share, 'models')

    fleet = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'hospital_fleet.launch.py')))

    def spawn(model, name, x, y, z, yaw=0.0):
        return Node(package='ros_gz_sim', executable='create', output='screen',
                    arguments=['-world', WORLD_ENTITY,
                               '-file', os.path.join(models, model, 'model.sdf'),
                               '-name', name, '-x', str(x), '-y', str(y),
                               '-z', str(z), '-Y', str(yaw)])

    last_ns = ROBOTS[-1][0]
    prop_gate = Node(
        package='pickplace_arm_bringup', executable='wait_for',
        name='wait_for_fleet_props', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['--label', 'fleet props', '--timeout', '900',
                   '--clock-stable', '0.5',
                   '--topic', f'/{last_ns}/diff_drive_controller/odom'])

    tables = []
    dx, dy, dyaw = delivery_table_pose()
    tables.append(spawn('rack_table', 'table_delivery', dx, dy, TABLE_SPAWN_Z, dyaw))
    for name, _colour, (tx, ty, tyaw), _rack, _stand in collection_points():
        tables.append(spawn('rack_table', f'table_{name}', tx, ty,
                            TABLE_SPAWN_Z, tyaw))
    assert len(tables) == len(STATIC_TABLES)

    racks = [spawn(f'rack_{colour}', f'rack_{colour}', rx, ry, RACK_SPAWN_Z)
             for _name, colour, _table, (rx, ry), _stand in collection_points()]

    release = Node(
        package='pickplace_arm_bringup', executable='rack_release',
        name='rack_release', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[a for ns in ARM_ROBOTS for a in ('--robot', ns)]
        + [a for _n, colour, _t, _r, _s in collection_points()
           for a in ('--model', f'rack_{colour}')]
        + ['--duration', '20', '--rate', '2'])
    detaches = [release]

    # --- the job -------------------------------------------------------------
    mission_gate = Node(
        package='pickplace_arm_bringup', executable='wait_for',
        name='wait_for_mission', output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['--label', 'mission', '--timeout', '900']
        + [a for ns in ARM_ROBOTS
           for a in ('--topic', f'/{ns}/nav_ready')])

    missions = [
        Node(package='pickplace_arm_bringup', executable='mission_delivery',
             namespace=ns, name='mission_delivery', output='screen',
             parameters=[{'use_sim_time': True}])
        for ns in ARM_ROBOTS
    ]
    manager = Node(
        package='pickplace_arm_bringup', executable='task_manager',
        name='task_manager', output='screen',
        parameters=[{'use_sim_time': True},
                    {'depart_stagger': ParameterValue(
                        LaunchConfiguration('depart_stagger'),
                        value_type=float)}])


    return LaunchDescription([
        DeclareLaunchArgument(
            'depart_stagger', default_value='20.0',
            description='Seconds between one robot leaving reception and the '
                        'next being dispatched.'),
        fleet,
        prop_gate,
        RegisterEventHandler(OnProcessExit(
            target_action=prop_gate,
            on_exit=tables
            # Racks spawn onto table tops, so the tables must exist first.
            + [TimerAction(period=10.0, actions=racks)]
            + [TimerAction(period=20.0, actions=detaches)]
            + [TimerAction(period=24.0, actions=[mission_gate])])),
        RegisterEventHandler(OnProcessExit(target_action=mission_gate,
                                           on_exit=missions + [manager])),
    ])
