#!/usr/bin/env python3
"""Full pick-and-place for the pickplace_arm in Gazebo Harmonic."""
import math
import time
import threading
from contextlib import contextmanager
from threading import Lock

import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration as RclDuration
from sensor_msgs.msg import PointCloud2, JointState
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PointStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Empty

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform support)

from pymoveit2 import MoveIt2

ARM_JOINTS = ['fr3_joint1', 'fr3_joint2', 'fr3_joint3',
              'fr3_joint4', 'fr3_joint5', 'fr3_joint6', 'fr3_joint7']
ROLL_JOINT_IDX = ()
ROLL_LIMIT = 2.0 * math.pi - 0.02
GRIPPER_JOINTS = ['fr3_finger_joint1']
GRASP_LINK = 'fr3_hand_tcp'
FINGER_LINKS = ['fr3_leftfinger', 'fr3_rightfinger', 'fr3_hand']

# --- task geometry (base_link frame) -----------------------------------------
BOX_ID = 'target_box'
BOX_SIZE = 0.06
GROUND_Z = -0.13228
FRONT_CAM_Z = 0.091 - GROUND_Z         # 0.22328 m above the floor
PLACE_XY = (0.70, 0.25)
GRASP_Z = GROUND_Z + BOX_SIZE / 2.0   # fingertip z when grasping a ground box
APPROACH_Z = GRASP_Z + 0.15           # pre-grasp / lift height
GRIP_OPEN = 0.038
GRIP_CLOSED = 0.0
GRIP_MOVE_TIME = 0.4

# --- fixed waits --------------------------------------------------------------
ARM_SETTLE = 0.15
SCENE_SYNC = 0.5
SCENE_SYNC_RELEASE = 0.3

PRECISE_VEL, PRECISE_ACC = 0.30, 0.25
FRICTION_VEL, FRICTION_ACC = 0.20, 0.20


def _duration(seconds):
    """builtin_interfaces/Duration from a float, for trajectory points."""
    return Duration(sec=int(seconds),
                    nanosec=int(round((seconds - int(seconds)) * 1e9)))
GRIP_HOLD = BOX_SIZE / 2.0 - 0.001     # 0.029 for the 0.06 box
BOX_COLORS = ('red', 'green', 'blue')

# --- grasp verification -------------------------------------------------------
FINGER_HELD_MIN = 0.015
MAX_GRASP_ATTEMPTS = 3

CARRY_POSITION = (0.50, 0.00, GROUND_Z + 0.65)
LIDAR_SCAN_Z = 0.3143            # laser centre above the floor
CARRY_LIDAR_CLEARANCE = 0.17     # payload underside to scan plane


def carry_height(payload_grip_height):
    """Fingertip height above the floor that keeps the payload out of the lidar scan."""
    return max(0.65, LIDAR_SCAN_Z + CARRY_LIDAR_CLEARANCE + payload_grip_height)

HOME_CONFIG = [0.0, 0.5012, 0.0, -1.9509, 0.0, 2.452, 0.7854]

GRIPPER_X = 0.70
GRIPPER_Y = 0.0
READY_Z = GROUND_Z + 0.50
LIFT_CLEARANCE = 0.12
WELD_CLEARANCE = 0.06
FRONT_X_OFFSET = BOX_SIZE / 2.0
MAX_REACH_X = 0.85

EXPECTED_BOX_Z = GROUND_Z + BOX_SIZE / 2.0

# --- perception ---------------------------------------------------------------
SCAN_POSITION = (0.60, 0.00, GROUND_Z + 0.70)
SCAN_PITCH = math.radians(173.6)

COLOR_HSV = {
    'blue':  [((95, 120, 60), (115, 255, 255))],
    'green': [((35, 80, 40), (85, 255, 255))],
    'red':   [((0, 100, 50), (10, 255, 255)), ((170, 100, 50), (180, 255, 255))],
}
HSV_LOWER = COLOR_HSV['blue'][0][0]
HSV_UPPER = COLOR_HSV['blue'][0][1]
MIN_VALID_PIXELS = 30


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def zdown_quat(yaw):
    """Gripper pointing straight down, yawed about world z (xyzw)."""
    cz, sz = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return qmul((0.0, 0.0, sz, cz), (1.0, 0.0, 0.0, 0.0))


def scan_quat(pitch, yaw=0.0):
    """Gripper tilted forward-down by `pitch` from horizontal, then yawed about world z (xyzw)."""
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    yaw_q = (0.0, 0.0, sy, cy)
    pitch_q = (0.0, sp, 0.0, cp)
    return qmul(yaw_q, pitch_q)


class PickAndPlace(Node):
    GRASP_MODELS = {c: f'box_{c}' for c in BOX_COLORS}
    PAYLOAD_GRIP_HEIGHT = BOX_SIZE / 2.0

    def __init__(self):
        super().__init__('pick_and_place')
        cbg = ReentrantCallbackGroup()

        ns = self.get_namespace().strip('/')
        self.tf_prefix = f'{ns}/' if ns else ''

        self.arm = MoveIt2(
            node=self, joint_names=ARM_JOINTS, base_link_name='base_link',
            end_effector_name=GRASP_LINK, group_name='arm', callback_group=cbg)
        self.arm.max_velocity = 0.70
        self.arm.max_acceleration = 0.55

        self.scan_position = SCAN_POSITION
        self.scan_pitch = SCAN_PITCH

        self.gripper_pub = self.create_publisher(
            JointTrajectory, 'gripper_controller/joint_trajectory', 10)

        # --- perception: point cloud subscriptions + TF ---
        self._cloud_lock = Lock()
        self._latest_cloud = None
        self.create_subscription(
            PointCloud2, 'camera/points', self._cloud_cb, 1,
            callback_group=cbg)
        self._front_lock = Lock()
        self._front_cloud = None
        self.create_subscription(
            PointCloud2, 'front_camera/points', self._front_cloud_cb, 1,
            callback_group=cbg)

        self._joint_pos = {}
        self.create_subscription(
            JointState, 'joint_states', self._joint_state_cb, 10,
            callback_group=cbg)

        self.tf_buffer = tf2_ros.Buffer(cache_time=RclDuration(seconds=30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._attach_pubs, self._detach_pubs = {}, {}
        for c, model in self.GRASP_MODELS.items():
            self._attach_pubs[c] = self.create_publisher(Empty, f'{model}/attach', 10)
            self._detach_pubs[c] = self.create_publisher(Empty, f'{model}/detach', 10)
        self._attached_color = None
        self.detach_box(log_label='startup')

        self.get_logger().info('Pick-and-place node ready')

    # --- rigid grasp ---------------------------------------------------------
    def _publish_box_cmd(self, pub, wait_for_bridge=True):
        """Publish an Empty to a gz DetachableJoint topic."""
        deadline = time.time() + 5.0
        while wait_for_bridge and pub.get_subscription_count() == 0 and time.time() < deadline:
            time.sleep(0.1)
        for _ in range(3):
            pub.publish(Empty())
            time.sleep(0.05)

    def attach_box(self, color):
        """Weld the `color` box to the gripper."""
        if color not in self._attach_pubs:
            self.get_logger().warn(f'[attach] no attach topic for colour {color}')
            return
        self.gripper(GRIP_HOLD, 'settle jaws onto the box faces', release=False)
        self._publish_box_cmd(self._attach_pubs[color])
        self._attached_color = color
        self.get_logger().info(f'[attach] {color} box welded to the gripper')

    def detach_box(self, color=None, log_label=''):
        """Release the weld."""
        colors = [color] if color else list(self._detach_pubs)
        for c in colors:
            self._publish_box_cmd(self._detach_pubs[c])
        if self._attached_color or log_label:
            self.get_logger().info(
                f'[attach] released {color or "all boxes"} {log_label}'.strip())
        self._attached_color = None

    def _cloud_cb(self, msg):
        with self._cloud_lock:
            self._latest_cloud = msg

    def _front_cloud_cb(self, msg):
        with self._front_lock:
            self._front_cloud = msg

    def _joint_state_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self._joint_pos[name] = pos

    def grasp_is_holding(self):
        """True if a box is currently pinched between the jaws."""
        if self._attached_color:
            return True
        if not self._joint_pos:
            return False
        gap = max(self._joint_pos.get(j, 0.0) for j in GRIPPER_JOINTS)
        return gap > FINGER_HELD_MIN


    # --- primitives ----------------------------------------------------------
    def move_pose(self, x, y, z, yaw=0.0, cartesian=False, label='',
                  quat_xyzw=None, strict=False):
        self.get_logger().info(
            f'[arm] -> ({x:.2f},{y:.2f},{z:.2f}) yaw={yaw:.2f} '
            f'{"cartesian " if cartesian else ""}{label}')
        if quat_xyzw is None:
            quat_xyzw = zdown_quat(yaw)
        if cartesian:
            self.arm.move_to_pose(position=(x, y, z), quat_xyzw=quat_xyzw,
                                  cartesian=True, cartesian_fraction_threshold=0.0)
            ok = self.arm.wait_until_executed()
        else:
            ok = self._move_pose_direct(x, y, z, quat_xyzw, label, strict)
        if not ok:
            self.get_logger().warn(f'[arm] motion failed: {label}')
        time.sleep(ARM_SETTLE)
        return ok

    def _move_pose_direct(self, x, y, z, quat_xyzw, label, strict=False):
        """Pose move that solves IK from the current joint state and plans in joint space."""
        sol = self.arm.compute_ik(position=(x, y, z), quat_xyzw=quat_xyzw)
        cfg = self._extract_arm_config(sol) if sol is not None else None
        if cfg is not None:
            cfg = self._normalize_roll_config(cfg)
            self.arm.move_to_configuration(cfg)
            if self.arm.wait_until_executed():
                return True
            self.get_logger().warn(f'[arm] direct joint move failed for {label}'
                                   + ('' if strict else '; trying pose plan'))
        else:
            self.get_logger().warn(f'[arm] IK seed failed for {label}'
                                   + ('' if strict else '; using pose plan'))
        if strict:
            return False
        self.arm.move_to_pose(position=(x, y, z), quat_xyzw=quat_xyzw,
                              cartesian=False, cartesian_fraction_threshold=0.0)
        return self.arm.wait_until_executed()

    @staticmethod
    def _extract_arm_config(joint_state):
        """Pull the ARM_JOINTS positions (in order) out of an IK JointState."""
        try:
            return [joint_state.position[joint_state.name.index(j)]
                    for j in ARM_JOINTS]
        except (ValueError, IndexError):
            return None

    def _current_arm_config(self):
        if not all(j in self._joint_pos for j in ARM_JOINTS):
            return None
        return [self._joint_pos[j] for j in ARM_JOINTS]

    def _normalize_roll_config(self, config):
        """Replace each roll joint's target with the equivalent angle closest to the current one."""
        cur = self._current_arm_config()
        if cur is None:
            return config
        config = list(config)
        for i in ROLL_JOINT_IDX:
            best = config[i]
            for alt in (config[i] - 2.0 * math.pi, config[i] + 2.0 * math.pi):
                if -ROLL_LIMIT <= alt <= ROLL_LIMIT and abs(alt - cur[i]) < abs(best - cur[i]):
                    best = alt
            config[i] = best
        return config

    @contextmanager
    def slow_arm(self, velocity=FRICTION_VEL, acceleration=FRICTION_ACC):
        """Run one arm move at reduced scaling, then restore the previous values."""
        prev_v, prev_a = self.arm.max_velocity, self.arm.max_acceleration
        self.arm.max_velocity = velocity
        self.arm.max_acceleration = acceleration
        try:
            yield
        finally:
            self.arm.max_velocity = prev_v
            self.arm.max_acceleration = prev_a

    def _wait_gripper(self, target, timeout_sec=2.0, reach_tol=0.002):
        """Block until the jaws have finished moving, or the timeout."""
        time.sleep(GRIP_MOVE_TIME + 0.05)
        deadline = time.time() + timeout_sec
        prev, stable = None, 0
        while time.time() < deadline:
            pos = self._joint_pos.get(GRIPPER_JOINTS[0])
            if pos is not None:
                if abs(pos - target) <= reach_tol:
                    return True
                if prev is not None and abs(pos - prev) <= 0.0005:
                    stable += 1
                    if stable >= 3:
                        return True
                else:
                    stable = 0
                prev = pos
            time.sleep(0.1)
        self.get_logger().warn(
            f'[gripper] jaws never settled at {target:.3f} within '
            f'{timeout_sec:.1f}s (last {prev})')
        return False

    def move_config(self, config, label=''):
        """Move to an explicit joint configuration."""
        config = self._normalize_roll_config(config)
        self.get_logger().info(f'[arm] -> configuration {label}')
        self.arm.move_to_configuration(config)
        ok = self.arm.wait_until_executed()
        if not ok:
            self.get_logger().warn(f'[arm] motion failed: config {label}')
        time.sleep(ARM_SETTLE)
        return ok

    def gripper(self, pos, label='', release=None):
        """Command the jaws to `pos`."""
        self.get_logger().info(f'[gripper] -> {pos} {label}')
        if release is None:
            release = pos > GRIP_CLOSED and self._attached_color
        m = JointTrajectory()
        m.joint_names = GRIPPER_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(pos)] * len(GRIPPER_JOINTS)
        pt.time_from_start = _duration(GRIP_MOVE_TIME)
        m.points = [pt]
        for _ in range(3):
            self.gripper_pub.publish(m)
            time.sleep(0.05)
        self._wait_gripper(float(pos))
        if release:
            self.detach_box(log_label=f'on gripper open ({label})')

    def tf_frame(self, name):
        """A TF frame name for this robot."""
        return name if name == 'map' else self.tf_prefix + name

    def add_box(self, xy, z_center=None):
        if z_center is None:
            z_center = GROUND_Z + BOX_SIZE / 2.0  # ground box in base_link frame
        self.arm.add_collision_box(
            id=BOX_ID, size=(BOX_SIZE, BOX_SIZE, BOX_SIZE),
            position=(xy[0], xy[1], z_center), quat_xyzw=(0.0, 0.0, 0.0, 1.0),
            frame_id='base_link')
        time.sleep(SCENE_SYNC)

    # --- perception ------------------------------------------------------------
    def detect_box_pose(self, timeout_sec=5.0, debug_save=False, color='blue'):
        """Wrist (eye-in-hand) detection; the arm must already be at the scan pose."""
        return self._detect('wrist', timeout_sec, debug_save, color)

    def detect_box_front(self, timeout_sec=2.0, debug_save=False, color='blue',
                         gate=None):
        """Base-mounted front camera detection, used while driving."""
        return self._detect('front', timeout_sec, debug_save, color, gate)

    def _detect(self, source, timeout_sec, debug_save=False, color='blue',
                gate=None):
        """HSV-segment the `color` box in a fresh cloud; centroid (x, y, z) in base_link or None."""
        log = self.get_logger()
        if source == 'front':
            lock, cloud_frame = self._front_lock, self.tf_frame('front_camera_link')
        else:
            lock, cloud_frame = self._cloud_lock, self.tf_frame('camera_link')
        with lock:
            if source == 'front':
                self._front_cloud = None
            else:
                self._latest_cloud = None

        deadline = time.time() + timeout_sec
        cloud = None
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            with lock:
                cloud = self._front_cloud if source == 'front' else self._latest_cloud
            if cloud is not None:
                break
        if cloud is None:
            log.error('[detect] no point cloud received before timeout')
            return None

        h, w = cloud.height, cloud.width
        if h <= 1:
            log.error('[detect] point cloud is not organized (height <= 1)')
            return None

        pts = point_cloud2.read_points(
            cloud, field_names=('x', 'y', 'z', 'rgb'), skip_nans=False)
        x = pts['x'].reshape(h, w)
        y = pts['y'].reshape(h, w)
        z = pts['z'].reshape(h, w)
        rgb_u32 = pts['rgb'].copy().view(np.uint32)
        r = ((rgb_u32 >> 16) & 0xFF).reshape(h, w).astype(np.uint8)
        g = ((rgb_u32 >> 8) & 0xFF).reshape(h, w).astype(np.uint8)
        b = (rgb_u32 & 0xFF).reshape(h, w).astype(np.uint8)
        rgb_img = np.dstack([r, g, b])

        hsv = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)
        mask = None
        for lo, hi in COLOR_HSV.get(color, COLOR_HSV['blue']):
            m = cv2.inRange(hsv, lo, hi)
            mask = m if mask is None else cv2.bitwise_or(mask, m)

        if debug_save:
            cv2.imwrite('/tmp/box_rgb_debug.png', cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR))
            cv2.imwrite('/tmp/box_mask_debug.png', mask)

        valid = (mask.astype(bool) & np.isfinite(x) & np.isfinite(y) & np.isfinite(z))
        if gate is not None:
            gx0, gx1, gy0, gy1, gz0, gz1 = gate
            valid = valid & (x >= gx0) & (x <= gx1) & (y >= gy0) & (y <= gy1) \
                          & (z >= gz0) & (z <= gz1)
        n_valid = int(valid.sum())
        if n_valid < MIN_VALID_PIXELS:
            log.error(f'[detect] only {n_valid} valid {color} pixels found (need '
                      f'>= {MIN_VALID_PIXELS}){" in gate" if gate else ""} '
                      f'-- box not found')
            return None

        cx, cy, cz = float(x[valid].mean()), float(y[valid].mean()), float(z[valid].mean())
        log.info(f'[detect] {n_valid} px -> centroid ({cx:.3f},{cy:.3f},{cz:.3f}) '
                  f'in {cloud.header.frame_id}')

        point = PointStamped()
        point.header = cloud.header
        point.header.frame_id = cloud_frame
        point.point.x, point.point.y, point.point.z = cx, cy, cz
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tf_frame('base_link'), cloud_frame, rclpy.time.Time(),
                timeout=RclDuration(seconds=1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            log.error(f'[detect] TF lookup base_link <- {cloud_frame} failed: {e}')
            return None
        point_base = tf2_geometry_msgs.do_transform_point(point, tf)
        bx, by, bz = point_base.point.x, point_base.point.y, point_base.point.z

        if abs(bz - EXPECTED_BOX_Z) > 0.02:
            log.warn(f'[detect] detected z={bz:.3f} differs from expected ground '
                      f'box z={EXPECTED_BOX_Z:.3f} by more than 2cm')

        log.info(f'[detect] box in base_link: ({bx:.3f}, {by:.3f}, {bz:.3f})')
        return (bx, by, bz)

    # --- sequence ------------------------------------------------------------
    def _attempt_grasp(self, bx, by):
        """One pre-grasp, descend and close on the box at (bx, by)."""
        log = self.get_logger()

        if not self.move_pose(bx, by, APPROACH_Z, 0.0, label='pre-grasp'):
            log.warn('[grasp] pre-grasp unreachable -- box too far for a clean grasp')
            return False

        self.arm.remove_collision_object(BOX_ID)
        time.sleep(SCENE_SYNC_RELEASE)
        with self.slow_arm(PRECISE_VEL, PRECISE_ACC):
            self.move_pose(bx, by, GRASP_Z, 0.0, cartesian=True, label='descend')
        self.gripper(GRIP_CLOSED, 'grasp')

        if not self.grasp_is_holding():
            log.warn('[grasp] jaws closed on air (no box between fingers)')
            return False
        log.info('[grasp] box held between the jaws')
        return True

    def pick_up_box(self):
        """Wrist-cam scan, grasp and lift, then hold the box in the carry pose."""
        log = self.get_logger()
        log.info('=== PICK UP: START ===')
        sx, sy, sz = self.scan_position

        for attempt in range(1, MAX_GRASP_ATTEMPTS + 1):
            log.info(f'--- grasp attempt {attempt}/{MAX_GRASP_ATTEMPTS} ---')

            self.gripper(GRIP_OPEN, 'open')
            self.move_pose(sx, sy, sz, label='scan',
                           quat_xyzw=scan_quat(self.scan_pitch))
            time.sleep(0.2)
            detection = self.detect_box_pose()
            if detection is None:
                log.warn(f'[pick] no box detected at scan pose (attempt {attempt})')
                continue
            bx, by, _bz = detection
            box_xy = (bx, by)

            # box known to MoveIt for pre-grasp/transport awareness + RViz
            self.add_box(box_xy)

            # 1+2) pre-grasp -> descend -> close, and verify the jaws hold it
            if not self._attempt_grasp(bx, by):
                self.arm.remove_collision_object(BOX_ID)
                continue

            # 3) attach the box so MoveIt carries it (and RViz shows it grasped)
            self.add_box(box_xy, z_center=GROUND_Z + BOX_SIZE / 2.0)
            self.arm.attach_collision_object(
                id=BOX_ID, link_name=GRASP_LINK, touch_links=FINGER_LINKS)
            time.sleep(SCENE_SYNC)

            self.move_pose(bx, by, APPROACH_Z, 0.0, cartesian=True, label='lift')
            cx, cy, _ = CARRY_POSITION
            cz = GROUND_Z + carry_height(self.PAYLOAD_GRIP_HEIGHT)
            carry_ok = self.move_pose(cx, cy, cz, 0.0, cartesian=False,
                                      label='carry', strict=True)

            # 5) confirm the box survived the lift + carry (didn't slip out).
            if not carry_ok or not self.grasp_is_holding():
                log.warn('[pick] box slipped during lift/carry -- retrying')
                self.arm.detach_collision_object(BOX_ID)
                self.arm.remove_collision_object(BOX_ID)
                if not carry_ok:
                    self.gripper(GRIP_OPEN, 'release-after-failed-carry')
                continue

            log.info('=== PICK UP: DONE (box held) ===')
            return True

        log.error(f'=== PICK UP: FAILED after {MAX_GRASP_ATTEMPTS} attempts '
                  f'(no box grasped) ===')
        self.arm.remove_collision_object(BOX_ID)
        return False

    def grab_below(self, grasp_z=GRASP_Z, color='blue', x_offset=FRONT_X_OFFSET):
        """Descend straight onto the `color` box that claw_approach has driven under the gripper."""
        log = self.get_logger()
        log.info('=== CLAW GRAB: descend straight down ===')
        self.gripper(GRIP_OPEN, 'open')
        det = self.detect_box_front(timeout_sec=1.5, color=color)
        if det is None:
            log.warn('[claw] box not seen for grab')
            return False
        want_x = det[0] + x_offset
        bx = min(MAX_REACH_X, want_x)
        if want_x > MAX_REACH_X:
            log.warn(
                f'[claw] REACH CAP BIT: wanted {want_x:.3f} (face {det[0]:.3f} '
                f'+ {x_offset:.3f}), capped to {MAX_REACH_X:.3f} -- the jaws '
                f'will close {want_x - MAX_REACH_X:.3f} m SHORT of the '
                f'payload centre, i.e. on its near edge. Fix the approach '
                f'distance, not this cap.')
        by = det[1]
        with self.slow_arm(PRECISE_VEL, PRECISE_ACC):
            self.move_pose(bx, by, grasp_z, 0.0, cartesian=True,
                           label='claw descend', quat_xyzw=zdown_quat(0.0))
        self.gripper(GRIP_CLOSED, 'grasp')
        if not self.grasp_is_holding():
            log.warn('[claw] jaws closed on air -- lifting to retry')
            self.move_pose(bx, by, READY_Z, 0.0, cartesian=True,
                           label='claw lift-empty', quat_xyzw=zdown_quat(0.0))
            return False
        log.info('[claw] box held between the jaws')

        self.add_box((bx, by), z_center=grasp_z)
        self.arm.attach_collision_object(
            id=BOX_ID, link_name=GRASP_LINK, touch_links=FINGER_LINKS)
        time.sleep(SCENE_SYNC)
        with self.slow_arm(FRICTION_VEL, FRICTION_ACC):
            if not self.move_pose(bx, by, grasp_z + WELD_CLEARANCE, 0.0,
                                  cartesian=True, label='break contact',
                                  quat_xyzw=zdown_quat(0.0)):
                log.warn('[claw] break contact failed -- retrying once before '
                         'the weld')
                self.move_pose(bx, by, grasp_z + WELD_CLEARANCE, 0.0,
                               cartesian=True, label='break contact (retry)',
                               quat_xyzw=zdown_quat(0.0))

        if not self.grasp_is_holding():
            log.warn('[claw] box slipped during break-contact')
            self.arm.detach_collision_object(BOX_ID)
            self.arm.remove_collision_object(BOX_ID)
            self.gripper(GRIP_OPEN, 'release-after-slip')
            return False

        self.attach_box(color)

        lift_z = max(READY_Z, grasp_z + LIFT_CLEARANCE)
        self.move_pose(bx, by, lift_z, 0.0, cartesian=True,
                       label='claw lift', quat_xyzw=zdown_quat(0.0))

        cx, cy, _ = CARRY_POSITION
        cz = GROUND_Z + carry_height(self.PAYLOAD_GRIP_HEIGHT)
        if not self.move_pose(cx, cy, cz, 0.0, cartesian=False, label='carry',
                              strict=True):
            log.warn('[claw] carry move failed -- releasing so the retry '
                     'starts from a clean, empty-gripper state')
            self.arm.detach_collision_object(BOX_ID)
            self.arm.remove_collision_object(BOX_ID)
            self.detach_box(log_label='after failed carry')
            self.gripper(GRIP_OPEN, 'release-after-failed-carry')
            return False
        log.info('=== CLAW GRAB: DONE (box held) ===')
        return True

    def place_box_down(self, place_xy=None):
        """Place the held box at place_xy (base_link frame), release, and return home."""
        log = self.get_logger()
        px, py = place_xy if place_xy is not None else PLACE_XY
        place_yaw = math.atan2(py, px)
        log.info('=== PLACE DOWN: START ===')

        self.move_pose(px, py, APPROACH_Z, place_yaw, cartesian=False, label='to place')
        self.move_pose(px, py, GRASP_Z, place_yaw, cartesian=True, label='place-down')
        self.arm.detach_collision_object(BOX_ID)
        self.arm.remove_collision_object(BOX_ID)
        time.sleep(SCENE_SYNC_RELEASE)
        self.gripper(GRIP_OPEN, 'release')

        # retreat and go home
        self.move_pose(px, py, APPROACH_Z, place_yaw, cartesian=True, label='retreat')
        self.move_config(HOME_CONFIG, 'home')
        log.info('=== PLACE DOWN: DONE ===')

    def run(self):
        """Stationary pick-and-place: pick the box up and place it at PLACE_XY."""
        self.get_logger().info('=== PICK AND PLACE: START ===')
        if not self.pick_up_box():
            return
        self.place_box_down()
        self.get_logger().info('=== PICK AND PLACE: DONE ===')


def main():
    rclpy.init()
    node = PickAndPlace()
    ex = rclpy.executors.MultiThreadedExecutor(4)
    ex.add_node(node)
    t = threading.Thread(target=node.run, daemon=True)
    # give MoveIt/action servers a moment, then run the sequence
    time.sleep(3.0)
    t.start()
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
