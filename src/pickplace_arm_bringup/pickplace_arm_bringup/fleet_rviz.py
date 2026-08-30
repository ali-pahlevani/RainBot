"""Generate one RViz config for a whole fleet, from the fleet's robot list."""
import os
import tempfile

import yaml

COLOURS = ['255; 60; 60', '60; 255; 60', '60; 160; 255', '255; 200; 0']

_SENSOR_QOS = {'Depth': 5, 'Durability Policy': 'Volatile',
               'History Policy': 'Keep Last',
               'Reliability Policy': 'Best Effort'}
_RELIABLE_QOS = {'Depth': 5, 'Durability Policy': 'Volatile',
                 'History Policy': 'Keep Last',
                 'Reliability Policy': 'Reliable'}
_LATCHED_QOS = {'Depth': 5, 'Durability Policy': 'Transient Local',
                'History Policy': 'Keep Last',
                'Reliability Policy': 'Reliable'}


def _with_filter(qos):
    """Topic blocks on displays carry a Filter size; tool topics do not."""
    return dict(qos, **{'Filter size': 10})


def _grid():
    return {
        'Alpha': 0.5, 'Cell Size': 1, 'Class': 'rviz_default_plugins/Grid',
        'Color': '100; 100; 100', 'Enabled': True,
        'Line Style': {'Line Width': 0.03, 'Value': 'Lines'},
        'Name': 'Grid', 'Normal Cell Count': 0,
        'Offset': {'X': 0, 'Y': 0, 'Z': 0}, 'Plane': 'XY',
        'Plane Cell Count': 60, 'Reference Frame': '<Fixed Frame>',
        'Value': True,
    }


def _map(name, topic, scheme, alpha, enabled):
    return {
        'Alpha': alpha, 'Class': 'rviz_default_plugins/Map',
        'Color Scheme': scheme, 'Draw Behind': True, 'Enabled': enabled,
        'Name': name,
        'Topic': _with_filter(_LATCHED_QOS) | {'Value': topic},
        'Update Topic': _with_filter(_RELIABLE_QOS) | {'Value': topic + '_updates'},
        'Use Timestamp': False, 'Value': enabled,
    }


def _tf(enabled):
    return {
        'Class': 'rviz_default_plugins/TF', 'Enabled': enabled,
        'Filter (blacklist)': '', 'Filter (whitelist)': '',
        'Frame Timeout': 15, 'Frames': {'All Enabled': True},
        'Marker Scale': 0.5, 'Name': 'TF', 'Show Arrows': True,
        'Show Axes': True, 'Show Names': False, 'Tree': {},
        'Update Interval': 0, 'Value': enabled,
    }


def _robot_model(ns):
    return {
        'Alpha': 1, 'Class': 'rviz_default_plugins/RobotModel',
        'Collision Enabled': False, 'Description File': '',
        'Description Source': 'Topic',
        'Description Topic': dict(_LATCHED_QOS, **{'Value': f'/{ns}/robot_description'}),
        'Enabled': True, 'Links': {'All Links Enabled': True,
                                   'Expand Joint Details': False,
                                   'Expand Link Details': False,
                                   'Expand Tree': False,
                                   'Link Tree Style': 'Links in Alphabetic Order'},
        'Mass Properties': {'Inertia': False, 'Mass': False},
        'Name': f'{ns} model', 'TF Prefix': ns, 'Update Interval': 0,
        'Value': True, 'Visual Enabled': True,
    }


def _laser(ns, colour):
    return {
        'Alpha': 1, 'Autocompute Intensity Bounds': True,
        'Autocompute Value Bounds': {'Max Value': 10, 'Min Value': -10,
                                     'Value': True},
        'Axis': 'Z', 'Channel Name': 'intensity',
        'Class': 'rviz_default_plugins/LaserScan', 'Color': colour,
        'Color Transformer': 'FlatColor', 'Decay Time': 0, 'Enabled': True,
        'Invert Rainbow': False, 'Max Color': '255; 255; 255',
        'Max Intensity': 4096, 'Min Color': '0; 0; 0', 'Min Intensity': 0,
        'Name': f'{ns} LIDAR', 'Position Transformer': 'XYZ',
        'Selectable': True, 'Size (Pixels)': 3, 'Size (m)': 0.04,
        'Style': 'Points',
        'Topic': _with_filter(_SENSOR_QOS) | {'Value': f'/{ns}/scan'},
        'Use Fixed Frame': True, 'Use rainbow': True, 'Value': True,
    }


def _odometry(ns, colour):
    return {
        'Angle Tolerance': 0.1, 'Class': 'rviz_default_plugins/Odometry',
        'Covariance': {
            'Orientation': {'Alpha': 0.5, 'Color': '255; 255; 127',
                            'Color Style': 'Unique', 'Frame': 'Local',
                            'Offset': 1, 'Scale': 1, 'Value': True},
            'Position': {'Alpha': 0.3, 'Color': '204; 51; 204', 'Scale': 1,
                         'Value': True},
            'Value': False},
        'Enabled': True, 'Keep': 60, 'Name': f'{ns} odometry (EKF)',
        'Position Tolerance': 0.1,
        'Shape': {'Alpha': 0.6, 'Axes Length': 1, 'Axes Radius': 0.1,
                  'Color': colour, 'Head Length': 0.08, 'Head Radius': 0.06,
                  'Shaft Length': 0.15, 'Shaft Radius': 0.03,
                  'Value': 'Arrow'},
        'Topic': _with_filter(_RELIABLE_QOS) | {'Value': f'/{ns}/odometry/filtered'},
        'Value': True,
    }


def _particles(ns, colour):
    return {
        'Alpha': 0.5, 'Arrow Length': 0.2, 'Axes Length': 0.3,
        'Axes Radius': 0.01, 'Class': 'rviz_default_plugins/PoseArray',
        'Color': colour, 'Enabled': True, 'Head Length': 0.07,
        'Head Radius': 0.03, 'Name': f'{ns} AMCL particles',
        'Shaft Length': 0.23, 'Shaft Radius': 0.01, 'Shape': 'Arrow (Flat)',
        'Topic': _with_filter(_SENSOR_QOS) | {'Value': f'/{ns}/particle_cloud'},
        'Value': True,
    }


def _path(name, topic, colour, width):
    return {
        'Alpha': 1, 'Buffer Length': 1, 'Class': 'rviz_default_plugins/Path',
        'Color': colour, 'Enabled': True, 'Head Diameter': 0.3,
        'Head Length': 0.2, 'Length': 0.3, 'Line Style': 'Lines',
        'Line Width': width, 'Name': name,
        'Offset': {'X': 0, 'Y': 0, 'Z': 0}, 'Pose Color': '255; 85; 255',
        'Pose Style': 'None', 'Radius': 0.03, 'Shaft Diameter': 0.1,
        'Shaft Length': 0.1,
        'Topic': _with_filter(_RELIABLE_QOS) | {'Value': topic},
        'Value': True,
    }


def _footprint(ns, colour):
    return {
        'Alpha': 1, 'Class': 'rviz_default_plugins/Polygon', 'Color': colour,
        'Enabled': True, 'Name': f'{ns} footprint',
        'Topic': _with_filter(_RELIABLE_QOS) | {
            'Value': f'/{ns}/local_costmap/published_footprint'},
        'Value': True,
    }


def _image(name, topic, enabled):
    return {
        'Class': 'rviz_default_plugins/Image', 'Enabled': enabled,
        'Max Value': 1, 'Median window': 5, 'Min Value': 0, 'Name': name,
        'Normalize Range': True,
        'Topic': dict(_SENSOR_QOS, **{'Value': topic}), 'Value': enabled,
    }


def _cloud(name, topic, enabled):
    return {
        'Alpha': 1, 'Autocompute Intensity Bounds': True,
        'Autocompute Value Bounds': {'Max Value': 10, 'Min Value': -10,
                                     'Value': True},
        'Axis': 'Z', 'Channel Name': 'intensity',
        'Class': 'rviz_default_plugins/PointCloud2', 'Color': '255; 255; 255',
        'Color Transformer': 'RGB8', 'Decay Time': 0, 'Enabled': enabled,
        'Invert Rainbow': False, 'Max Color': '255; 255; 255',
        'Max Intensity': 4096, 'Min Color': '0; 0; 0', 'Min Intensity': 0,
        'Name': name, 'Position Transformer': 'XYZ', 'Selectable': True,
        'Size (Pixels)': 3, 'Size (m)': 0.01, 'Style': 'Points',
        'Topic': _with_filter(_SENSOR_QOS) | {'Value': topic},
        'Use Fixed Frame': True, 'Use rainbow': True, 'Value': enabled,
    }


def _robot_group(idx, ns, arm, nav):
    """Everything belonging to one robot, in one collapsible group."""
    colour = COLOURS[idx % len(COLOURS)]
    displays = [_robot_model(ns), _laser(ns, colour)]

    if nav:
        displays += [
            _odometry(ns, colour),
            _particles(ns, colour),
            _footprint(ns, colour),
            _path(f'{ns} global plan', f'/{ns}/plan', '0; 255; 0', 0.04),
            _path(f'{ns} local plan', f'/{ns}/local_plan', '255; 0; 255', 0.03),
            _map(f'{ns} local costmap', f'/{ns}/local_costmap/costmap',
                 'costmap', 0.6, True),
            _map(f'{ns} global costmap', f'/{ns}/global_costmap/costmap',
                 'costmap', 0.4, False),
        ]

    if arm:
        displays += [
            _image(f'{ns} front camera', f'/{ns}/front_camera/image', False),
            _image(f'{ns} wrist camera', f'/{ns}/camera/image', False),
            _cloud(f'{ns} front cloud', f'/{ns}/front_camera/points', False),
            _cloud(f'{ns} wrist cloud', f'/{ns}/camera/points', False),
        ]

    role = ', '.join(['nav'] * nav + ['arm'] * arm) or 'parked'
    return {'Class': 'rviz_common/Group', 'Displays': displays,
            'Enabled': True, 'Name': f'{ns} ({role})'}


def _tool_topic(topic):
    return dict(_RELIABLE_QOS, **{'Value': topic})


def fleet_rviz_config(robots, arm_robots=(), nav_robots=(),
                      tool_ns=None, view_centre=(0.0, 0.0)):
    """Write an RViz config for `robots` and return its path."""
    names = [ns for ns, *_ in robots]
    tool_ns = tool_ns or (list(nav_robots)[0] if nav_robots else names[0])

    displays = [
        _grid(),
        {'Class': 'rviz_default_plugins/MarkerArray', 'Enabled': True,
         'Name': 'Task manager', 'Namespaces': {}, 'Queue Size': 100,
         'Topic': dict(_with_filter(_RELIABLE_QOS), Value='/fleet/markers'),
         'Value': True},
        _map('Map (shared)', '/map', 'map', 0.7, True),
        _tf(False),
    ]
    displays += [
        _robot_group(idx, ns, ns in arm_robots, ns in nav_robots)
        for idx, (ns, *_) in enumerate(robots)
    ]

    config = {
        'Panels': [
            {'Class': 'rviz_common/Displays', 'Help Height': 78,
             'Name': 'Displays',
             'Property Tree Widget': {
                 'Expanded': ['/Global Options1'] +
                             [f'/{ns}1' for ns in names[:2]],
                 'Splitter Ratio': 0.5},
             'Tree Height': 620},
            {'Class': 'rviz_common/Selection', 'Name': 'Selection'},
            {'Class': 'rviz_common/Tool Properties',
             'Expanded': ['/2D Goal Pose1'], 'Name': 'Tool Properties',
             'Splitter Ratio': 0.58},
            {'Class': 'rviz_common/Views', 'Expanded': ['/Current View1'],
             'Name': 'Views', 'Splitter Ratio': 0.5},
            {'Class': 'rviz_common/Time', 'Experimental': False,
             'Name': 'Time', 'SyncMode': 0,
             'SyncSource': f'{names[0]} LIDAR'},
        ],
        'Visualization Manager': {
            'Class': '',
            'Displays': displays,
            'Enabled': True,
            'Global Options': {'Background Color': '48; 48; 48',
                               'Fixed Frame': 'map', 'Frame Rate': 30},
            'Name': 'root',
            'Tools': [
                {'Class': 'rviz_default_plugins/Interact',
                 'Hide Inactive Objects': True},
                {'Class': 'rviz_default_plugins/MoveCamera'},
                {'Class': 'rviz_default_plugins/Select'},
                {'Class': 'rviz_default_plugins/FocusCamera'},
                {'Class': 'rviz_default_plugins/Measure', 'Line color': '128; 128; 0'},
                {'Class': 'rviz_default_plugins/SetInitialPose',
                 'Covariance x': 0.25, 'Covariance y': 0.25,
                 'Covariance yaw': 0.068,
                 'Topic': _tool_topic(f'/{tool_ns}/initialpose')},
                {'Class': 'rviz_default_plugins/SetGoal',
                 'Topic': _tool_topic(f'/{tool_ns}/goal_pose')},
                {'Class': 'rviz_default_plugins/PublishPoint',
                 'Single click': True,
                 'Topic': _tool_topic('/clicked_point')},
            ],
            'Transformation': {'Current': {'Class': 'rviz_default_plugins/TF'}},
            'Value': True,
            'Views': {
                'Current': {
                    'Class': 'rviz_default_plugins/Orbit', 'Distance': 26.0,
                    'Enable Stereo Rendering': {
                        'Stereo Eye Separation': 0.06,
                        'Stereo Focal Distance': 1, 'Swap Stereo Eyes': False,
                        'Value': False},
                    'Focal Point': {'X': float(view_centre[0]),
                                    'Y': float(view_centre[1]), 'Z': 0.0},
                    'Focal Shape Fixed Size': True, 'Focal Shape Size': 0.05,
                    'Invert Z Axis': False, 'Name': 'Current View',
                    'Near Clip Distance': 0.01, 'Pitch': 1.2,
                    'Target Frame': 'map',
                    'Value': 'Orbit (rviz_default_plugins)', 'Yaw': 3.14},
                'Saved': [
                    {'Class': 'rviz_default_plugins/Orbit', 'Distance': 6.0,
                     'Enable Stereo Rendering': {
                         'Stereo Eye Separation': 0.06,
                         'Stereo Focal Distance': 1,
                         'Swap Stereo Eyes': False, 'Value': False},
                     'Focal Point': {'X': 0, 'Y': 0, 'Z': 0},
                     'Focal Shape Fixed Size': True, 'Focal Shape Size': 0.05,
                     'Invert Z Axis': False, 'Name': f'Chase {ns}',
                     'Near Clip Distance': 0.01, 'Pitch': 0.6,
                     'Target Frame': f'{ns}/base_link',
                     'Value': 'Orbit (rviz_default_plugins)', 'Yaw': 3.1}
                    for ns in names
                ],
            },
        },
        'Window Geometry': {
            'Displays': {'collapsed': False},
            'Height': 1000, 'Hide Left Dock': False, 'Hide Right Dock': True,
            'Selection': {'collapsed': False},
            'Time': {'collapsed': False},
            'Tool Properties': {'collapsed': False},
            'Views': {'collapsed': True},
            'Width': 1800, 'X': 60, 'Y': 30,
        },
    }

    out = os.path.join(tempfile.mkdtemp(prefix='fleet_rviz_'), 'fleet.rviz')
    with open(out, 'w') as fh:
        yaml.safe_dump(config, fh, default_flow_style=False, sort_keys=True)
    return out
