"""Rewrite a single-robot Nav2/AMCL parameter file for one namespaced robot."""
import os
import tempfile

import yaml

# value -> what it becomes for robot <ns>. Frames first, then topics.
FRAME_VALUES = ('base_link', 'base_footprint', 'odom')
TOPIC_VALUES = ('/scan', 'scan', '/imu',
                '/diff_drive_controller/odom',
                '/diff_drive_controller/cmd_vel_unstamped')


BLOCK_NAME_KEYS = ('observation_sources',)


def _rewrite(value, ns):
    if not isinstance(value, str):
        return value
    if value in FRAME_VALUES:
        return f'{ns}/{value}'
    if value in TOPIC_VALUES:
        return f'/{ns}{value}' if value.startswith('/') else f'/{ns}/{value}'
    return value


def _walk(node, ns):
    if isinstance(node, dict):
        out = {k: (v if k in BLOCK_NAME_KEYS else _walk(v, ns))
               for k, v in node.items()}
        if 'static_layer' in out and isinstance(out['static_layer'], dict):
            out['static_layer']['map_topic'] = '/map'
        if 'global_frame' in out and 'robot_base_frame' in out \
                and 'plugins' in out:
            out['transform_tolerance'] = 1.0
            out['always_send_full_costmap'] = False
        if 'base_frame_id' in out and 'odom_frame_id' in out:
            out['transform_tolerance'] = 0.5
        return out
    if isinstance(node, list):
        return [_walk(v, ns) for v in node]
    return _rewrite(node, ns)


def namespaced_params(source, ns, overrides=None):
    """Return a path to `source` rewritten for robot `ns`."""
    with open(source) as fh:
        data = yaml.safe_load(fh)

    data = _walk(data, ns)
    for node_name, params in (overrides or {}).items():
        data.setdefault(node_name, {}).setdefault('ros__parameters', {}).update(params)

    out = os.path.join(tempfile.mkdtemp(prefix=f'{ns}_params_'),
                       os.path.basename(source))
    with open(out, 'w') as fh:
        yaml.safe_dump({ns: data}, fh, default_flow_style=False)
    return out


def diff_drive_frames(ns):
    """Write the per-robot frame overlay for diff_drive_controller."""
    out = os.path.join(tempfile.mkdtemp(prefix=f'{ns}_ddframes_'),
                       f'diff_drive_frames_{ns}.yaml')
    with open(out, 'w') as fh:
        yaml.safe_dump({f'/{ns}/diff_drive_controller': {'ros__parameters': {
            'odom_frame_id': f'{ns}/odom',
            'base_frame_id': f'{ns}/base_link',
            }}}, fh)
    return out
