"""Where the fleet stands in aws_hospital.sdf."""
import math
import os

# --- the reception desk -------------------------------------------------------
DESK_BOUNDS = (-3.50, 3.50, 2.80, 6.30)     # xmin, xmax, ymin, ymax
DESK_FACE_Y = 6.30                          # the front (lobby-facing) edge
DESK_FACE = (0.0, DESK_FACE_Y)              # centre of that edge

# --- the formation ------------------------------------------------------------
FORMATION_CENTRE = (0.0, 8.75)
FORMATION_RADIUS = 2.00
FORMATION_ROTATION = math.pi / 2.0      # apex points north, away from the desk


def _vertex(k):
    """Vertex k of the formation triangle, k in 0..2."""
    a = FORMATION_ROTATION + k * 2.0 * math.pi / 3.0
    return (FORMATION_CENTRE[0] + FORMATION_RADIUS * math.cos(a),
            FORMATION_CENTRE[1] + FORMATION_RADIUS * math.sin(a))


def _facing(p, target):
    """Yaw that points a robot standing at p at `target`."""
    return math.atan2(target[1] - p[1], target[0] - p[0])


ROBOTS = [
    ('r1',) + _vertex(0) + (_facing(_vertex(0), DESK_FACE),),
    ('r2',) + _vertex(1) + (_facing(_vertex(1), DESK_FACE),),
    ('r3',) + _vertex(2) + (_facing(_vertex(2), DESK_FACE),),
]

# --- running a smaller fleet ---------------------------------------------------
_want = os.environ.get('FLEET_ROBOTS')
if _want:
    _keep = [n.strip() for n in _want.split(',') if n.strip()]
    _known = {n for n, *_ in ROBOTS}
    _unknown = [n for n in _keep if n not in _known]
    if _unknown:
        raise RuntimeError(
            f'FLEET_ROBOTS names {_unknown}, not in the fleet {sorted(_known)}')
    ROBOTS = [r for r in ROBOTS if r[0] in _keep]

NAV_ROBOTS = tuple(ns for ns, *_ in ROBOTS)
ARM_ROBOTS = tuple(ns for ns, *_ in ROBOTS)

SPAWN_Z = 0.14
WORLD_ENTITY = 'aws_hospital'

ROBOT_FOOTPRINT = (0.494, -0.496, 0.335)
ROBOT_RADIUS = 0.598


# --- where they go when the job is done ---------------------------------------
PARKING_CENTRE = (1.0, 14.25)
PARKING_RADIUS = 2.00
PARKING_ROTATION = math.pi / 2.0


def parking_vertices():
    """The three parking poses, as (x, y, yaw), in claim order."""
    out = []
    for k in range(3):
        a = PARKING_ROTATION + k * 2.0 * math.pi / 3.0
        px = PARKING_CENTRE[0] + PARKING_RADIUS * math.cos(a)
        py = PARKING_CENTRE[1] + PARKING_RADIUS * math.sin(a)
        out.append((px, py, math.atan2(PARKING_CENTRE[1] - py,
                                       PARKING_CENTRE[0] - px)))
    return out


def robot(ns):
    """The (x, y, yaw) of one robot by namespace."""
    for name, x, y, yaw in ROBOTS:
        if name == ns:
            return (x, y, yaw)
    raise KeyError(f'{ns} is not in the fleet: {[r[0] for r in ROBOTS]}')
