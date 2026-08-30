"""Where the rack tables stand in aws_hospital.sdf, and where the racks sit on them."""
import math

# --- the table model ----------------------------------------------------------
TABLE_LONG = 1.327
TABLE_SHORT = 0.668
TABLE_TOP = 0.3238
TABLE_NEAR_FACE = 0.330
TABLE_SPAWN_Z = 0.0

# --- where a rack stands on a table -------------------------------------------
RACK_EDGE_INSET = 0.15
RACK_LOCAL_Y = -(TABLE_SHORT / 2.0 - RACK_EDGE_INSET)      # -0.184
RACK_SPAWN_Z = TABLE_TOP
RACK_GRIP_HEIGHT = 0.170

# --- where the robot stands ---------------------------------------------------
NAV_STANDOFF = 1.30
STANDOFF_LOCAL_Y = RACK_LOCAL_Y - NAV_STANDOFF             # -1.484

DELIVERY_SLOT_LOCAL_Y = -0.065
DELIVERY_NAV_STANDOFF = 0.81
DELIVERY_STANDOFF_LOCAL_Y = DELIVERY_SLOT_LOCAL_Y - DELIVERY_NAV_STANDOFF


def _to_world(table, lx, ly):
    """A point in a table's local frame, in world coordinates."""
    tx, ty, tyaw = table
    c, s = math.cos(tyaw), math.sin(tyaw)
    return (tx + c * lx - s * ly, ty + s * lx + c * ly)


def _robot_yaw(table):
    """Heading of a robot working at this table: it faces the table's local +y."""
    return table[2] + math.pi / 2.0


# --- the collection points ----------------------------------------------------
COLLECTION_TABLES = [
    ('collect_0', 'red',   (-8.90,  -5.00, math.pi)),      # west wing, north end
    ('collect_1', 'green', (8.50, -19.50, 0.0)),           # east wing, south
    ('collect_2', 'blue',  (-7.50, -26.50, 3.0 * math.pi / 2)),   # far south hall
]

# --- the delivery point -------------------------------------------------------
DELIVERY_TABLE = ('delivery', (-3.50, 10.00, math.pi))

DELIVERY_SLOT_SPACING = 0.38
DELIVERY_SLOT_LOCAL_X = (-DELIVERY_SLOT_SPACING, 0.0, DELIVERY_SLOT_SPACING)


# --- derived poses -------------------------------------------------------------
def collection_points():
    """[(name, colour, table_pose, rack_xy, robot_pose)] for the three pickups."""
    out = []
    for name, colour, table in COLLECTION_TABLES:
        rack = _to_world(table, 0.0, RACK_LOCAL_Y)
        stand = _to_world(table, 0.0, STANDOFF_LOCAL_Y)
        out.append((name, colour, table, rack, stand + (_robot_yaw(table),)))
    return out


def delivery_slots():
    """[(index, slot_xy, robot_pose)] for the three rack drop points, left to right."""
    _, table = DELIVERY_TABLE
    out = []
    for i, lx in enumerate(DELIVERY_SLOT_LOCAL_X):
        slot = _to_world(table, lx, DELIVERY_SLOT_LOCAL_Y)
        stand = _to_world(table, lx, DELIVERY_STANDOFF_LOCAL_Y)
        out.append((i, slot, stand + (_robot_yaw(table),)))
    return out


def delivery_standoff(slot_index=None):
    """The pose a robot drives to in order to use the delivery table."""
    _, table = DELIVERY_TABLE
    lx = 0.0 if slot_index is None else DELIVERY_SLOT_LOCAL_X[slot_index]
    return _to_world(table, lx, DELIVERY_STANDOFF_LOCAL_Y) + (_robot_yaw(table),)


def delivery_table_pose():
    return DELIVERY_TABLE[1]


# --- where robots wait for the delivery table ---------------------------------
DELIVERY_HOLD_RADIUS = 3.00

DELIVERY_HOLD_BEARINGS = (
    math.radians(-116.0),      # index 0 -- r1, arriving from collect_0
    math.radians(-43.0),       # index 1 -- r2, arriving from collect_1
    math.radians(-85.0),       # index 2 -- r3, arriving from collect_2
)


def delivery_hold_pose(index):
    """Where robot `index` waits for its turn at the delivery table."""
    tx, ty, _ = DELIVERY_TABLE[1]
    a = DELIVERY_HOLD_BEARINGS[index % len(DELIVERY_HOLD_BEARINGS)]
    ux, uy = math.cos(a), math.sin(a)
    return (tx + ux * DELIVERY_HOLD_RADIUS,
            ty + uy * DELIVERY_HOLD_RADIUS,
            math.atan2(-uy, -ux))


# --- what belongs in the map --------------------------------------------------
STATIC_TABLES = [t for _, _, t in COLLECTION_TABLES] + [DELIVERY_TABLE[1]]
