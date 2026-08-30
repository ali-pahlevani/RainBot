#!/usr/bin/env python3
"""One robot's half of the sample-transport job: collect, deliver, park."""
import math
import sys
import threading
import time

import numpy as np
import rclpy
import tf2_ros
from sensor_msgs.msg import LaserScan
from sensor_msgs_py import point_cloud2
from rclpy.duration import Duration as RclDuration
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

from pickplace_arm_bringup.mission_2 import Mission2Hospital, TIGHT_YAW_TOLERANCE, \
    DEFAULT_YAW_TOLERANCE
from pickplace_arm_bringup.pick_and_place import (
    BOX_ID, BOX_SIZE, GRIP_OPEN, GROUND_Z, HOME_CONFIG, PRECISE_ACC,
    PRECISE_VEL, SCENE_SYNC_RELEASE, zdown_quat)
from pickplace_arm_bringup import rack_table_layout as RT


class DeliveryMission(Mission2Hospital):
    """Collect one rack, deliver it to a slot, park."""
    DELIVERY_PLACE_Z = GROUND_Z + RT.TABLE_TOP + RT.RACK_GRIP_HEIGHT

    ARM_BASE_X = 0.0799          # fr3_link0 ahead of base_link
    ARM_BASE_Z = GROUND_Z + 0.3837
    ARM_REACH = 0.855
    ARM_REACH_FRACTION = 0.95

    def _max_x_for(self, py, pz):
        """Largest base_link x the arm can reach at this y and z, or None."""
        budget = (self.ARM_REACH * self.ARM_REACH_FRACTION) ** 2 \
            - py ** 2 - (pz - self.ARM_BASE_Z) ** 2
        if budget <= 0.0:
            return None
        return self.ARM_BASE_X + math.sqrt(budget)

    def __init__(self):
        super().__init__()
        self.ns = self.get_namespace().strip('/')
        self._task = None
        self._task_lock = threading.Lock()
        self._scan_lock = threading.Lock()
        self._scan = None
        self.create_subscription(
            LaserScan, 'scan', self._scan_cb,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))
        self._fail_reason = ''
        task_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(String, 'task', self._on_task, task_qos)
        self.status_pub = self.create_publisher(String, 'mission_status', 10)
        self._claim_node = rclpy.create_node(
            'claim_client', namespace=self.get_namespace())
        self._slot_client = self._claim_node.create_client(Trigger, 'claim_slot')
        self._park_client = self._claim_node.create_client(Trigger, 'claim_park')
        self.get_logger().info(f'[{self.ns}] delivery mission ready, waiting for a task')

    # --- talking to the task manager -----------------------------------------
    def _on_task(self, msg):
        with self._task_lock:
            self._task = msg.data

    def take_task(self):
        with self._task_lock:
            t, self._task = self._task, None
        return t

    def say(self, state, detail=''):
        """Report progress."""
        self.status_pub.publish(String(data=f'{self.ns}|{state}|{detail}'))
        self.get_logger().info(f'[{self.ns}] {state} {detail}')

    def _fail(self, reason):
        """Record why the errand ended and return False."""
        self._fail_reason = reason
        self.get_logger().error(f'[{self.ns}] errand failed: {reason}')
        return False

    def _claim(self, client, what, wait_sec=0.0):
        """Ask the manager for a slot or a parking vertex."""
        if not client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error(f'[{self.ns}] no {what} service -- is the '
                                    f'task manager running?')
            return None
        deadline = time.time() + max(wait_sec, 0.0)
        announced = False
        last_said = 0.0
        while True:
            fut = client.call_async(Trigger.Request())
        # Safe: _claim_node is spun by nobody else.
            rclpy.spin_until_future_complete(self._claim_node, fut,
                                             timeout_sec=30.0)
            if not fut.done() or fut.result() is None:
                self.get_logger().error(f'[{self.ns}] {what} request timed out')
                return None
            res = fut.result()
            if res.success:
                return int(res.message)
            if 'in use' in res.message and time.time() < deadline:
                now = time.time()
                if not announced or now - last_said >= 15.0:
                    waited = now - (deadline - max(wait_sec, 0.0))
                    self.get_logger().info(
                        f'[{self.ns}] queued for the delivery table, waiting '
                        f'{waited:.0f}s ({res.message})')
                    announced = True
                    last_said = now
                time.sleep(2.0)
                continue
            self.get_logger().error(f'[{self.ns}] {what} refused: {res.message}')
            return None

    # --- placing on the delivery table ---------------------------------------
    def _slot_in_base_link(self, slot_xy):
        """Where a delivery slot is in the robot's own frame, right now."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tf_frame('base_link'), 'map', rclpy.time.Time(),
                timeout=RclDuration(seconds=2.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'[{self.ns}] base_link <- map failed: {e}')
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * slot_xy[0] - s * slot_xy[1] + t.x,
                s * slot_xy[0] + c * slot_xy[1] + t.y)

    FACE_Z_LO = GROUND_Z + 0.08
    FACE_Z_HI = GROUND_Z + 0.26
    FACE_HALF_WIDTH = 0.20      # m either side of the bow
    FACE_X_MIN, FACE_X_MAX = 0.25, 1.30

    def _table_face_ahead(self, timeout_sec=2.0):
        """Measured distance from base_link to the delivery table's front face."""
        log = self.get_logger()
        frame = self.tf_frame('front_camera_link')
        with self._front_lock:
            self._front_cloud = None
        deadline = time.time() + timeout_sec
        cloud = None
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            with self._front_lock:
                cloud = self._front_cloud
            if cloud is not None:
                break
        if cloud is None:
            log.warn(f'[{self.ns}] no front cloud to measure the table with')
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tf_frame('base_link'), frame, rclpy.time.Time(),
                timeout=RclDuration(seconds=1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            log.warn(f'[{self.ns}] TF for the table measurement failed: {e}')
            return None
        pts = point_cloud2.read_points(cloud, field_names=('x', 'y', 'z'),
                                       skip_nans=False)
        t = tf.transform.translation
        bx = np.asarray(pts['x'], dtype=float).ravel() + t.x
        by = np.asarray(pts['y'], dtype=float).ravel() + t.y
        bz = np.asarray(pts['z'], dtype=float).ravel() + t.z
        keep = (np.isfinite(bx) & np.isfinite(by) & np.isfinite(bz)
                & (np.abs(by) <= self.FACE_HALF_WIDTH)
                & (bz >= self.FACE_Z_LO) & (bz <= self.FACE_Z_HI)
                & (bx >= self.FACE_X_MIN) & (bx <= self.FACE_X_MAX))
        n = int(keep.sum())
        if n < 200:
            log.warn(f'[{self.ns}] only {n} points on the table face -- '
                     f'not measuring it')
            return None
        face = float(np.percentile(bx[keep], 10.0))
        log.info(f'[{self.ns}] delivery table face measured at base_link '
                 f'x={face:.3f} ({n} points)')
        return face

    FACE_TARGET = (RT.DELIVERY_NAV_STANDOFF - RT.DELIVERY_SLOT_LOCAL_Y
                   - RT.TABLE_NEAR_FACE)
    FACE_TOL = 0.04           # m; inside this, do not bother moving
    FACE_STEP_MAX = 0.45      # m; a bigger correction than this is a bad reading
    FACE_MIN_SAFE = 0.52      # m; never drive so the face is nearer than this
    FACE_ARRIVED_MAX = 1.20

    CREEP_SPEED = 0.10

    def _creep_blind(self, dist):
        """Drive `dist` metres forward (negative = back), open loop."""
        if abs(dist) < 0.01:
            return
        v = self.CREEP_SPEED if dist > 0 else -self.CREEP_SPEED
        self._drive_blind(v, abs(dist) / self.CREEP_SPEED)
        self._stop_base()

    def _scan_cb(self, msg):
        with self._scan_lock:
            self._scan = msg

    TABLE_WIDTH_LO, TABLE_WIDTH_HI = 1.15, 1.50
    FACE_PLANE_TOL = 0.05
    FACE_SEARCH_TOL = 0.40
    PARTIAL_WIDTH_MIN = 0.80
    NEAR_EDGE_MAX_BEARING = 65.0

    @staticmethod
    def _near_edge_for(slot_index):
        """Which end of the table face is well seen: +1 left, -1 right, 0 neither."""
        if slot_index is None:
            return 0
        lx = RT.DELIVERY_SLOT_LOCAL_X[slot_index]
        if abs(lx) < 1e-6:
            return 0
        return 1 if lx < 0.0 else -1

    def _table_centre_offset(self, face_x, near_edge=0):
        """Measured lateral offset from the bow to the delivery table's centre."""
        log = self.get_logger()
        with self._scan_lock:
            scan = self._scan
        if scan is None:
            log.warn(f'[{self.ns}] no scan to find the table centre with')
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tf_frame('base_link'), scan.header.frame_id,
                rclpy.time.Time(), timeout=RclDuration(seconds=1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            log.warn(f'[{self.ns}] TF for the table centre failed: {e}')
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        xx, yy, zz, ww = q.x, q.y, q.z, q.w
        r00 = 1.0 - 2.0 * (yy * yy + zz * zz)
        r01 = 2.0 * (xx * yy - zz * ww)
        r10 = 2.0 * (xx * yy + zz * ww)
        r11 = 1.0 - 2.0 * (xx * xx + zz * zz)
        rng = np.asarray(scan.ranges, dtype=float)
        ang = scan.angle_min + np.arange(rng.size) * scan.angle_increment
        good = np.isfinite(rng) & (rng >= scan.range_min) & (rng <= scan.range_max)
        rng = np.where(good, rng, 0.0)
        lx = rng * np.cos(ang)
        ly = rng * np.sin(ang)
        bx = r00 * lx + r01 * ly + t.x
        by = r10 * lx + r11 * ly + t.y
        near = (good
                & (np.abs(bx - face_x) <= self.FACE_SEARCH_TOL)
                & (np.abs(by) <= RT.TABLE_LONG))
        if int(near.sum()) < 25:
            log.warn(f'[{self.ns}] only {int(near.sum())} lidar returns near '
                     f'the table face at x={face_x:.3f} -- not measuring the '
                     f'centre')
            return None
        fx, fy = bx[near], by[near]
        mx, my = float(fx.mean()), float(fy.mean())
        u, _sv, _vt = np.linalg.svd(np.stack([fx - mx, fy - my]))
        dirx, diry = float(u[0, 0]), float(u[1, 0])
        # Perpendicular distance to the fitted line, not to a constant x.
        perp = np.abs(-diry * (bx - mx) + dirx * (by - my))
        on_face = good & near & (perp <= self.FACE_PLANE_TOL)
        n = int(on_face.sum())
        if n < 25:
            log.warn(f'[{self.ns}] only {n} lidar returns on the table face at '
                     f'x={face_x:.3f} -- not measuring the centre')
            return None
        s = dirx * (bx[on_face] - mx) + diry * (by[on_face] - my)
        lo, hi = float(s.min()), float(s.max())
        width = hi - lo
        if width > self.TABLE_WIDTH_HI:
            log.warn(f'[{self.ns}] table face measured {width:.3f} m wide '
                     f'({n} returns), wider than any table -- refusing the '
                     f'lateral correction')
            return None

        near_s = None
        if self.TABLE_WIDTH_LO <= width:
            method, centre_s = 'both edges', (lo + hi) / 2.0
        elif near_edge and width >= self.PARTIAL_WIDTH_MIN:
            near_s = hi if near_edge > 0 else lo
            ex = mx + dirx * near_s
            ey = my + diry * near_s
            bearing = abs(math.degrees(math.atan2(ey - t.y, ex - t.x)))
            if bearing > self.NEAR_EDGE_MAX_BEARING:
                log.warn(f'[{self.ns}] table face measured {width:.3f} m wide '
                         f'({n} returns) and its near end sits {bearing:.0f} deg '
                         f'off the bow, at the edge of the sweep -- that is a '
                         f'cut, not an edge; refusing the lateral correction')
                return None
            method = f'near edge at {bearing:.0f} deg'
            centre_s = near_s - near_edge * RT.TABLE_LONG / 2.0
        else:
            log.warn(f'[{self.ns}] table face measured {width:.3f} m wide '
                     f'({n} returns), not the expected {RT.TABLE_LONG:.3f} and '
                     f'too short to measure from one edge -- refusing the '
                     f'lateral correction')
            return None
        centre = my + diry * centre_s
        yaw_deg = math.degrees(math.atan2(dirx, abs(diry) if diry else 1e-9))
        log.info(f'[{self.ns}] table centre measured at base_link y={centre:+.3f} '
                 f'from the {method} (face {width:.3f} m wide, {n} returns, '
                 f'{yaw_deg:+.1f} deg off square)')
        if diry < 0.0:
            dirx, diry = -dirx, -diry
        if diry < math.cos(math.radians(45.0)):
            log.warn(f'[{self.ns}] table face is {yaw_deg:+.1f} deg off square '
                     f'-- too far to correct a slot against; refusing the '
                     f'lateral correction')
            return None
        return (centre, dirx, diry)

    SLOT_BEHIND_FACE = RT.DELIVERY_SLOT_LOCAL_Y + RT.TABLE_NEAR_FACE

    def _slot_on_face(self, face_x, frame, slot_index):
        """Where slot `slot_index` is in base_link, in the measured face's own frame."""
        centre, dirx, diry = frame
        mx = face_x + dirx * (centre / diry)
        my = centre
        nx, ny = diry, -dirx
        s = -RT.DELIVERY_SLOT_LOCAL_X[slot_index]
        return (mx + s * dirx + self.SLOT_BEHIND_FACE * nx,
                my + s * diry + self.SLOT_BEHIND_FACE * ny)

    def _square_up_on_table(self, tries=3):
        """Drive the base until the delivery table's face is at FACE_TARGET."""
        log = self.get_logger()
        face = None
        for attempt in range(1, tries + 1):
            face = self._table_face_ahead()
            if face is None:
                return None
            err = face - self.FACE_TARGET
            if abs(err) <= self.FACE_TOL:
                log.info(f'[{self.ns}] squared up on the table: face at '
                         f'{face:.3f} m (target {self.FACE_TARGET:.3f})')
                return face
            step = max(-self.FACE_STEP_MAX, min(self.FACE_STEP_MAX, err))
            # Never let a correction take the bumper into the table.
            if face - step < self.FACE_MIN_SAFE:
                step = face - self.FACE_MIN_SAFE
            if abs(step) <= 0.01:
                return face
            log.info(f'[{self.ns}] table face at {face:.3f} m, want '
                     f'{self.FACE_TARGET:.3f} -- moving {step:+.3f} m '
                     f'({attempt}/{tries})')
            self._creep_blind(step)
            time.sleep(0.6)          # let the base settle before re-measuring
        face = self._table_face_ahead()
        if face is not None:
            log.warn(f'[{self.ns}] table face settled at {face:.3f} m against a '
                     f'target of {self.FACE_TARGET:.3f} -- placing from there')
        return face

    def _settled_slot(self, slot_xy, timeout_sec=25.0):
        """The slot's position in base_link, once the pose estimate has settled."""
        log = self.get_logger()
        time.sleep(1.0)
        deadline = time.time() + timeout_sec
        prev = None
        while time.time() < deadline:
            cur = self._slot_in_base_link(slot_xy)
            if cur is None:
                time.sleep(0.3)
                continue
            if prev is not None and math.hypot(cur[0] - prev[0],
                                               cur[1] - prev[1]) < 0.03:
                return cur
            prev = cur
            time.sleep(0.5)
        if prev is not None:
            log.warn(f'[{self.ns}] slot reading never settled -- using the last '
                     f'({prev[0]:+.3f},{prev[1]:+.3f})')
        return prev

    SLOT_MIN_X = 0.30            # m ahead; below this the robot is not in front
    SLOT_MAX_ABS_Y = 0.55        # m abeam; the three slots span only +/-0.30

    def _slot_reading_is_sane(self, target):
        """Is this slot reading consistent with standing at the standoff?"""
        px, py = target
        return px >= self.SLOT_MIN_X and abs(py) <= self.SLOT_MAX_ABS_Y

    def _close_in_on(self, slot_xy, px):
        """Nudge forward with Nav2 so the slot lands inside the arm's reach."""
        log = self.get_logger()
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', self.tf_frame('base_link'), rclpy.time.Time(),
                timeout=RclDuration(seconds=2.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            log.error(f'[{self.ns}] map <- base_link failed: {e}')
            return False
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        advance = min(0.5, max(0.0, px - 0.78))
        if advance <= 0.02:
            log.info(f'[{self.ns}] slot reads {px:.3f} m ahead, already inside '
                     f'the working distance -- not moving up')
            return True
        gx = t.x + advance * math.cos(yaw)
        gy = t.y + advance * math.sin(yaw)
        log.info(f'[{self.ns}] slot is {px:.3f} m ahead -- moving up '
                 f'{advance:.2f} m to ({gx:+.2f},{gy:+.2f})')
        return self.navigate_to(self.make_map_goal(gx, gy, yaw))

    def _range_to(self, xy):
        """How far this robot's base_link is from a map point, right now."""
        here = self._base_xy_in_map()
        if here is None:
            return None
        return math.hypot(here[0] - xy[0], here[1] - xy[1])

    STANDOFF_ARRIVED_TOL = 0.60

    def _drive_to_delivery_standoff(self, slot_index=None, attempts=3):
        """Get to the delivery standoff and verify the arrival."""
        log = self.get_logger()
        stand = RT.delivery_standoff(slot_index)
        for attempt in range(1, attempts + 1):
            ok = self._navigate_with_recovery(stand, 'the delivery table',
                                              attempts=2, tight_yaw=True)
            d = self._range_to(stand[:2])
            if ok and d is not None and d <= self.STANDOFF_ARRIVED_TOL:
                return True
            face = self._table_face_ahead()
            if face is not None and face <= self.FACE_ARRIVED_MAX \
                    and self._table_centre_offset(
                        face, self._near_edge_for(slot_index)) is not None:
                log.warn(f'[{self.ns}] map -> base_link puts the robot '
                         f'{"unknown" if d is None else format(d, ".2f")} m '
                         f'from the delivery standoff, but the camera and lidar '
                         f'both have the table face at {face:.3f} m -- '
                         f'believing the sensors')
                return True
            if ok and d is not None:
                log.warn(f'[{self.ns}] Nav2 reported the delivery standoff '
                         f'reached, but map -> base_link puts the robot '
                         f'{d:.2f} m away (tolerance '
                         f'{self.STANDOFF_ARRIVED_TOL:.2f}) and the camera '
                         f'cannot see the table -- not believing it')
            if attempt < attempts:
                self._relocalize('delivery standoff not actually reached')
        log.error(f'[{self.ns}] could not reach the delivery standoff')
        return False

    def _repark_at_standoff(self, why, slot_index=None):
        """Go back to the delivery standoff and face the table."""
        self.get_logger().warn(
            f'[{self.ns}] {why} -- re-parking at the delivery standoff')
        return self._drive_to_delivery_standoff(slot_index, attempts=2)

    def place_in_slot(self, slot_index, slot_xy):
        """Put the carried rack down in `slot_xy` (map frame) and let go."""
        log = self.get_logger()
        target = None

        sensed = self._slot_from_sensors_only(slot_index)
        if sensed is not None:
            log.info(f'[{self.ns}] slot {slot_index} measured from the camera '
                     f'and lidar at base_link ({sensed[0]:+.3f},'
                     f'{sensed[1]:+.3f})')
            if self._slot_reading_is_sane(sensed):
                return self._lower_into_slot(slot_index, *sensed)
            log.warn(f'[{self.ns}] measured slot {slot_index} is not in front '
                     f'of the robot -- falling back to the map estimate')
        else:
            log.warn(f'[{self.ns}] could not measure the table -- falling back '
                     f'to the map estimate')

        for attempt in range(3):
            target = self._settled_slot(slot_xy)
            if target is None:
                return False
            if not self._slot_reading_is_sane(target):
                log.warn(f'[{self.ns}] slot {slot_index} reads base_link '
                         f'({target[0]:+.3f},{target[1]:+.3f}), which is not a '
                         f'slot in front of the table -- treating this as a '
                         f'POSE problem, not a reach problem')
                if attempt == 2 or not self._repark_at_standoff(
                        f'slot {slot_index} reading is off the table',
                        slot_index):
                    sensed = self._slot_from_sensors_only(slot_index)
                    if sensed is not None:
                        log.warn(f'[{self.ns}] slot {slot_index} recovered from '
                                 f'the camera and lidar alone at base_link '
                                 f'({sensed[0]:+.3f},{sensed[1]:+.3f}) -- '
                                 f'placing on the sensors, not the map')
                        return self._lower_into_slot(slot_index, *sensed)
                    log.error(f'[{self.ns}] could not recover a usable slot '
                              f'reading for slot {slot_index}')
                    return False
                continue
            over_z = self.DELIVERY_PLACE_Z + BOX_SIZE / 2.0 + 0.04
            cap = self._max_x_for(target[1], over_z)
            if cap is not None and target[0] <= cap:
                break
            if attempt == 2:
                log.warn(f'[{self.ns}] still {target[0]:.3f} m out after '
                         f'{attempt + 1} approaches (reachable to '
                         f'{cap if cap is None else round(cap, 3)}) -- '
                         f'placing at the cap')
                break
            if not self._close_in_on(slot_xy, target[0]):
                log.warn(f'[{self.ns}] could not move up to the table')
                break
        px, py = target

        face = self._square_up_on_table()
        if face is not None:
            px = face + self.SLOT_BEHIND_FACE
            log.info(f'[{self.ns}] slot depth from the measured table face: '
                     f'{px:.3f} m')
            frame = self._table_centre_offset(
                face, self._near_edge_for(slot_index))
            if frame is not None:
                was_x, was_y = px, py
                px, py = self._slot_on_face(face, frame, slot_index)
                log.info(f'[{self.ns}] slot {slot_index} on the measured table '
                         f'face: ({px:+.3f},{py:+.3f}) m (map said '
                         f'({was_x:+.3f},{was_y:+.3f}), correction '
                         f'({px - was_x:+.3f},{py - was_y:+.3f}))')
        else:
            log.warn(f'[{self.ns}] could not measure the table face -- placing '
                     f'on the map estimate alone, which is what put a rack on '
                     f'the floor')
        target = (px, py)

        if not self._slot_reading_is_sane(target):
            log.error(f'[{self.ns}] refusing to place: slot {slot_index} reads '
                      f'({px:+.3f},{py:+.3f}) in base_link, which is not in '
                      f'front of the table')
            return False
        log.info(f'[{self.ns}] slot {slot_index} is at base_link '
                 f'({px:+.3f},{py:+.3f})')
        return self._lower_into_slot(slot_index, px, py)

    def _slot_from_sensors_only(self, slot_index):
        """Where slot `slot_index` is, from the camera and lidar alone."""
        log = self.get_logger()
        face = self._square_up_on_table()
        if face is None:
            log.warn(f'[{self.ns}] no camera view of the table face -- '
                     f'cannot place on the sensors')
            return None
        frame = self._table_centre_offset(
            face, self._near_edge_for(slot_index))
        if frame is not None:
            return self._slot_on_face(face, frame, slot_index)
        log.warn(f'[{self.ns}] no usable lidar view of the face -- placing '
                 f'slot {slot_index} STRAIGHT AHEAD, which is where its '
                 f'standoff points, with the base\'s own lateral error left in')
        return (face + self.SLOT_BEHIND_FACE, 0.0)

    def _lower_into_slot(self, slot_index, px, py):
        """Reach over the slot, set the rack down, let go and back off."""
        log = self.get_logger()

        over_z = self.DELIVERY_PLACE_Z + BOX_SIZE / 2.0 + 0.04
        cap = self._max_x_for(py, over_z)
        if cap is None:
            log.error(f'[{self.ns}] slot {slot_index} is {py:+.3f} m to the side '
                      f'-- outside the arm envelope at any distance')
            return False
        if px > cap:
            log.warn(f'[{self.ns}] slot {slot_index} reads {px:.3f} m ahead at '
                     f'y={py:+.3f}; the arm reaches {cap:.3f} m there -- placing '
                     f'at the cap, {px - cap:.3f} m short')
            px = cap

        over_z = self.DELIVERY_PLACE_Z + BOX_SIZE / 2.0 + 0.04
        if not self.move_pose(px, py, over_z, 0.0, label=f'over-slot-{slot_index}',
                              quat_xyzw=zdown_quat(0.0), strict=True):
            log.error(f'[{self.ns}] could not reach over slot {slot_index} '
                      f'(rack still held)')
            return False
        with self.slow_arm(PRECISE_VEL, PRECISE_ACC):
            lowered = self.move_pose(px, py, self.DELIVERY_PLACE_Z, 0.0,
                                     cartesian=True,
                                     label=f'lower-into-slot-{slot_index}',
                                     quat_xyzw=zdown_quat(0.0))
        if not lowered:
            log.error(f'[{self.ns}] could not lower into slot {slot_index} '
                      f'(rack still held)')
            return False

        self.arm.detach_collision_object(BOX_ID)
        self.arm.remove_collision_object(BOX_ID)
        time.sleep(SCENE_SYNC_RELEASE)
        self.gripper(GRIP_OPEN, 'release')
        self.move_pose(px, py, over_z, 0.0, cartesian=True, label='retreat',
                       quat_xyzw=zdown_quat(0.0))

        log.info(f'[{self.ns}] backing off the delivery table')
        self._drive_blind(-0.25, 4.0)
        self._stop_base()
        self.move_config(HOME_CONFIG, 'gripper-down ready')
        return True

    # --- the whole errand ----------------------------------------------------
    def _retreat_from_delivery(self):
        """Back off the delivery table and stand the arm up."""
        log = self.get_logger()
        try:
            self._drive_blind(-0.35, 4.0)
            self._stop_base()
            self.move_config(HOME_CONFIG, 'gripper-down ready')
        except Exception as exc:
            log.warn(f'[{self.ns}] retreat failed: {exc}')
        try:
            hold = RT.delivery_hold_pose(self._fleet_index())
            log.info(f'[{self.ns}] clearing the delivery table back to the '
                     f'holding ring at ({hold[0]:+.2f},{hold[1]:+.2f})')
            if not self._navigate_with_recovery(hold, 'the holding ring',
                                                attempts=2):
                log.warn(f'[{self.ns}] could not get back to the holding ring '
                         f'-- the next robot is about to be sent to a table '
                         f'this one may still be near')
        except Exception as exc:
            log.warn(f'[{self.ns}] retreat to the holding ring failed: {exc}')

    def _navigate_with_recovery(self, pose, what, attempts=3, tight_yaw=False):
        """navigate_to, but treat a refusal as a pose problem and re-localize."""
        for attempt in range(1, attempts + 1):
            if tight_yaw:
                self._set_yaw_goal_tolerance(TIGHT_YAW_TOLERANCE)
            ok = self.navigate_to(self.make_map_goal(*pose))
            if tight_yaw:
                self._set_yaw_goal_tolerance(DEFAULT_YAW_TOLERANCE)
            if ok:
                return True
            if attempt < attempts:
                self._relocalize(f'could not plan to {what}')
        return False

    def _base_xy_in_map(self, default=None):
        """This robot's (x, y) in the map frame, or `default` if TF is not there."""
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', self.tf_frame('base_link'), rclpy.time.Time(),
                timeout=RclDuration(seconds=2.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'[{self.ns}] map <- base_link failed: {e}')
            return default
        return (tf.transform.translation.x, tf.transform.translation.y)

    def _fleet_index(self):
        """This robot's position in ARM_ROBOTS, used to reserve one queue spot."""
        from pickplace_arm_bringup.fleet_layout import ARM_ROBOTS
        try:
            return ARM_ROBOTS.index(self.ns)
        except ValueError:
            return 0

    def _relocalize(self, why):
        """Rotate in place until AMCL has re-weighted against the laser."""
        log = self.get_logger()
        log.warn(f'[{self.ns}] relocalizing: {why}')
        self._drive_blind(-0.20, 2.5)
        for _ in range(8):
            self._rotate_step(math.pi / 4.0)
        self._stop_base()

    def _rack_in_view(self, colour, tries=6):
        """True if the front camera can see the rack from where the robot stopped."""
        self.move_config(HOME_CONFIG, 'gripper-down ready')
        for _ in range(tries):
            if self.detect_box_front(timeout_sec=0.5, color=colour) is not None:
                return True
        return False

    def _search_for_rack(self, colour):
        """Recovery for having stopped somewhere other than the standoff."""
        log = self.get_logger()
        log.warn(f'[{self.ns}] no {colour} rack in view -- searching')
        self._drive_blind(-0.15, 2.0)
        found = self._sweep_for_rack(colour)
        if not found:
            log.warn(f'[{self.ns}] full turn without seeing the {colour} rack')
        return found

    SWEEP_ANGULAR = 0.7

    def _sweep_for_rack(self, colour, sweep=2.0 * math.pi):
        """Rotate continuously, watching the camera, and stop facing the rack."""
        log = self.get_logger()
        twist = Twist()
        twist.angular.z = self.SWEEP_ANGULAR
        turned = 0.0
        t_prev = time.time()
        while turned < sweep and rclpy.ok():
            self.cmd_vel_pub.publish(twist)
            det = self.detect_box_front(timeout_sec=0.15, color=colour)
            now = time.time()
            turned += self.SWEEP_ANGULAR * (now - t_prev)
            t_prev = now
            if det is not None:
                self._stop_base()
                # Let the chassis stop rocking before the servo takes over.
                time.sleep(0.4)
                log.info(f'[{self.ns}] {colour} rack found after '
                         f'{math.degrees(turned):.0f} deg of sweep')
                return True
        self._stop_base()
        return False

    def run_errand(self, colour, table, rack_xy, stand):
        log = self.get_logger()

        self.LAYOUT_TABLE_APPROACH = stand
        self.LAYOUT_TABLE_GRASP_Z = GROUND_Z + RT.TABLE_TOP + RT.RACK_GRIP_HEIGHT

        self.say('collecting', f'{colour} at ({rack_xy[0]:.2f},{rack_xy[1]:.2f})')
        reached = False
        for attempt in range(1, 4):
            self._set_yaw_goal_tolerance(TIGHT_YAW_TOLERANCE)
            ok = self.navigate_to(self.make_map_goal(*stand))
            self._set_yaw_goal_tolerance(DEFAULT_YAW_TOLERANCE)
            if ok and self._rack_in_view(colour):
                reached = True
                break
            if attempt < 3:
                self._search_for_rack(colour)
        if not reached:
            return self._fail('could not reach the collection table')

        if not self.claw_pick(rack_xy, color=colour,
                              grasp_z=self.LAYOUT_TABLE_GRASP_Z,
                              x_offset=self.LAYOUT_TABLE_X_OFFSET,
                              face_first=False):
            return self._fail(f'could not pick the {colour} rack')
        self.say('carrying', colour)

        self._drive_blind(-0.20, 3.5)
        self._stop_base()

        hold_pose = RT.delivery_hold_pose(self._fleet_index())
        self.say('approaching', 'the delivery table')
        if not self._navigate_with_recovery(hold_pose, 'the holding ring'):
            self.get_logger().warn(
                f'[{self.ns}] could not reach the holding ring -- waiting '
                f'where it stands')
        self._stop_base()
        self.say('holding', f'{RT.DELIVERY_HOLD_RADIUS:.1f} m from the table')

        slot = self._claim(self._slot_client, 'slot', wait_sec=1800.0)
        if slot is None:
            return self._fail('no delivery slot available')
        slot_xy = RT.delivery_slots()[slot][1]
        self.say('delivering', f'slot {slot}')

        if not self._drive_to_delivery_standoff(slot):
            self._retreat_from_delivery()
            return self._fail('could not reach the delivery table')

        if not self.grasp_is_holding():
            self._retreat_from_delivery()
            return self._fail(f'dropped the {colour} rack during the carry')
        if not self.place_in_slot(slot, slot_xy):
            if self._put_the_rack_down(slot):
                self.say('delivered', f'slot {slot} (set down without a slot '
                                      f'reading)')
                return True
            self._retreat_from_delivery()
            return self._fail(f'could not place in slot {slot}')
        self.say('delivered', f'slot {slot}')
        return True

    def _put_the_rack_down(self, slot_index):
        """Last resort: set the rack on the table in front of the robot."""
        log = self.get_logger()
        if not self.grasp_is_holding():
            log.info(f'[{self.ns}] nothing in the jaws -- nothing to set down')
            return False
        face = self._square_up_on_table()
        if face is None:
            log.warn(f'[{self.ns}] cannot see the table to set the rack down '
                     f'on -- it stays in the jaws')
            return False
        log.warn(f'[{self.ns}] placement failed -- setting the rack down '
                 f'straight ahead on the measured table rather than carrying '
                 f'it away')
        return self._lower_into_slot(slot_index, face + self.SLOT_BEHIND_FACE,
                                     0.0)

    # --- clearing the floor -------------------------------------------------
    def _park(self, outcome, detail):
        """Drive to a parking vertex and stop, then restate the errand outcome."""
        self.say(outcome, detail)

        vertex = self._claim(self._park_client, 'park')
        if vertex is None:
            self.get_logger().warn(
                f'[{self.ns}] no parking vertex available -- staying put')
            return
        from pickplace_arm_bringup.fleet_layout import parking_vertices
        pose = parking_vertices()[vertex]
        self.say('parking', f'vertex {vertex}')
        try:
            if self._navigate_with_recovery(pose, f'parking vertex {vertex}'):
                note = f'{detail} -- parked at vertex {vertex}'.lstrip(' -')
            else:
                note = f'{detail} -- could not reach parking vertex {vertex}'.lstrip(' -')
        except Exception as exc:                       # noqa: BLE001
            self.get_logger().error(f'[{self.ns}] parking drive raised: {exc}')
            note = f'{detail} -- parking drive failed'.lstrip(' -')
        self.say(outcome, note)

    def run(self):
        """Wait for a task, run it, then idle."""
        if not self.wait_for_localization(timeout_sec=600.0):
            self.say('failed', 'never localized')
            return
        self.say('idle', 'waiting for a task')
        while rclpy.ok():
            task = self.take_task()
            if task is None:
                time.sleep(0.5)
                continue
            # colour|table_x,table_y,table_yaw|rack_x,rack_y|stand_x,stand_y,stand_yaw
            try:
                parts = task.split('|')
                colour = parts[0]
                table = tuple(float(v) for v in parts[1].split(','))
                rack = tuple(float(v) for v in parts[2].split(','))
                stand = tuple(float(v) for v in parts[3].split(','))
            except (IndexError, ValueError) as e:
                self.get_logger().error(f'[{self.ns}] unreadable task {task!r}: {e}')
                continue
            self._fail_reason = ''
            if self.run_errand(colour, table, rack, stand):
                outcome, detail = 'done', ''
            else:
                outcome, detail = 'failed', self._fail_reason
            self._park(outcome, detail)
            return


def main():
    rclpy.init()
    node = DeliveryMission()
    ex = rclpy.executors.MultiThreadedExecutor(4)
    ex.add_node(node)

    def task():
        time.sleep(3.0)
        node.run()

    threading.Thread(target=task, daemon=True).start()
    try:
        while rclpy.ok():
            try:
                ex.spin()
                break
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                node.get_logger().warn(f'[executor] recovered: {exc}')
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
