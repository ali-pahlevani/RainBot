"""Where the pick-and-place run stands in aws_hospital.sdf."""
import math

# --- where the whole arrangement sits -----------------------------------------
ANCHOR = (-4.00, 8.00)
YAW = math.pi / 2.0          # the run faces north, up the lobby


def _place(x, y):
    """A layout-frame point in world coordinates."""
    c, s = math.cos(YAW), math.sin(YAW)
    return (ANCHOR[0] + c * x - s * y, ANCHOR[1] + s * x + c * y)


def _pose(x, y, yaw=0.0):
    """A layout-frame pose in world coordinates."""
    wx, wy = _place(x, y)
    return (wx, wy, yaw + YAW)


# --- the props ----------------------------------------------------------------
TABLE_TOP = 0.30
TABLE = _pose(2.30, 0.0)
TABLE_Z = TABLE_TOP / 2.0

RACK_GRIP_HEIGHT = 0.170

RACKS = [
    ('red',   _place(2.30, -0.22)),
    ('green', _place(2.30,  0.00)),
    ('blue',  _place(2.30,  0.22)),
]
RACK_SPAWN_Z = TABLE_TOP

COLUMNS = [
    (0, 0.30, _place(-1.0, -0.45)),   # red
    (1, 0.40, _place(-1.0,  0.00)),   # green
    (2, 0.50, _place(-1.0,  0.45)),   # blue
]
COLUMN_YAW = math.pi + YAW

# --- the poses the robot drives to --------------------------------------------
SPAWN = (ANCHOR[0], ANCHOR[1], YAW)
TABLE_APPROACH = _pose(1.30, 0.0)
FINAL_POSE = _pose(0.0, -1.8)
PLACE_APPROACH_DIR = (math.cos(YAW), math.sin(YAW))
