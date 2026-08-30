#!/usr/bin/env python3
"""Mission 2: sort three coloured boxes from a table onto three matching columns."""
import math
import time
import threading

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from rclpy.duration import Duration as RclDuration
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters

from pickplace_arm_bringup.mission import Mission
from pickplace_arm_bringup.pick_and_place import (
    HOME_CONFIG, GRIPPER_X, GRIP_OPEN, BOX_COLORS, BOX_ID, BOX_SIZE, GRASP_LINK,
    FINGER_LINKS, EXPECTED_BOX_Z, GROUND_Z, FRONT_CAM_Z, MAX_REACH_X,
    zdown_quat)
from pickplace_arm_bringup.search_and_pick import (
    APPROACH_LINEAR_GAIN, APPROACH_LINEAR_MAX, APPROACH_LINEAR_MIN,
    APPROACH_ANGULAR_GAIN, APPROACH_ANGULAR_MAX)
from pickplace_arm_bringup.hospital_pickplace_layout import (
    COLUMNS as HOSP_COLUMNS,
    FINAL_POSE as HOSP_FINAL_POSE,
    PLACE_APPROACH_DIR as HOSP_PLACE_APPROACH_DIR,
    RACKS as HOSP_RACKS,
    RACK_GRIP_HEIGHT as HOSP_RACK_GRIP_HEIGHT,
    TABLE_APPROACH as HOSP_TABLE_APPROACH,
    TABLE_TOP as HOSP_TABLE_TOP)

# --- layout (map frame) -------------------------------------------------------
TABLE_APPROACH = (1.30, 0.0, 0.0)      # pose the robot drives to before picking
TABLE_TOP_ABOVE_GROUND = 0.30           # table-top height above the floor
TABLE_Z = GROUND_Z + TABLE_TOP_ABOVE_GROUND          # table-top, base_link frame
# Fingertip z to grasp a box standing on the table: the box centre.
TABLE_GRASP_Z = TABLE_Z + BOX_SIZE / 2.0
TABLE_X_OFFSET = 0.030
BOXES = [                               # (colour, box map position) in pick order
    ('red',   (2.30, -0.22)),
    ('green', (2.30,  0.00)),
    ('blue',  (2.30,  0.22)),
]
COLUMNS = [                             # (column id, height, column map x,y)
    (0, 0.30, (-1.0, -0.45)),  # red   -> column 1, 30 cm
    (1, 0.40, (-1.0,  0.00)),  # green -> column 2, 40 cm
    (2, 0.50, (-1.0,  0.45)),  # blue  -> column 3, 50 cm
]
FINAL_POSE = (0.0, -1.8, 0.0)

NAV_STANDOFF = 1.10
COLUMN_STOP_X = 0.65
COLUMN_Y_TOL = 0.025
COLUMN_X_OFFSET = 0.10

TIGHT_YAW_TOLERANCE = 0.20
DEFAULT_YAW_TOLERANCE = 0.5


class Mission2(Mission):
    # --- layout (map frame) ---------------------------------------------------
    LAYOUT_TABLE_APPROACH = TABLE_APPROACH
    LAYOUT_TABLE_GRASP_Z = TABLE_GRASP_Z
    LAYOUT_TABLE_X_OFFSET = TABLE_X_OFFSET
    LAYOUT_BOXES = BOXES
    LAYOUT_COLUMNS = COLUMNS
    LAYOUT_FINAL_POSE = FINAL_POSE
    COLUMN_DETECT_GATE = None
    COLUMN_STOP_X = COLUMN_STOP_X
    OVER_Z_CEILING = GROUND_Z + 0.80
    TALL_COLUMN_H = 99.0
    COLUMN_STOP_X_TALL = COLUMN_STOP_X
    OVER_Z_CEILING_TALL = GROUND_Z + 0.80
    COLUMN_X_OFFSET = COLUMN_X_OFFSET
    COLUMN_DEPTH_BIAS = 0.0
    COLUMN_PLACE_X = COLUMN_STOP_X + COLUMN_X_OFFSET
    PAYLOAD_GRIP_HEIGHT = BOX_SIZE / 2.0
    PAYLOAD_FLOOR_Z = EXPECTED_BOX_Z
    PLACE_APPROACH_DIR = (1.0, 0.0)

    def __init__(self):
        super().__init__()
        self._set_params_client = self.create_client(
            SetParameters, 'controller_server/set_parameters')
        self.get_logger().info('Mission 2 node ready')

    # --- Nav2 tuning ----------------------------------------------------------
    def _set_yaw_goal_tolerance(self, value, timeout_sec=3.0):
        """Set controller_server's goal-checker yaw tolerance at runtime."""
        log = self.get_logger()
        if not self._set_params_client.wait_for_service(timeout_sec=timeout_sec):
            log.warn('[nav] controller_server set_parameters service unavailable')
            return False
        req = SetParameters.Request()
        p = Parameter()
        p.name = 'general_goal_checker.yaw_goal_tolerance'
        p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=value)
        req.parameters = [p]
        future = self._set_params_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        ok = future.done() and future.result() is not None and future.result().results[0].successful
        if not ok:
            log.warn(f'[nav] failed to set yaw_goal_tolerance={value}')
        return ok

    def _stop_x_for(self, height):
        """Front-cam stop distance for a column of this height."""
        if height is not None and height >= self.TALL_COLUMN_H:
            return self.COLUMN_STOP_X_TALL
        return self.COLUMN_STOP_X

    def _base_in_odom(self, timeout_sec=0.05):
        """Base position in the odom frame."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.tf_frame('odom'), self.tf_frame('base_link'),
                rclpy.time.Time(),
                timeout=RclDuration(seconds=timeout_sec))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'[creep] odom<-base_link TF failed: {e}')
            return None
        t = tf.transform.translation
        return t.x, t.y

    def _creep_forward(self, dist, speed=0.12, timeout_sec=25.0):
        """Drive straight forward by `dist` metres, measured with odom TF."""
        log = self.get_logger()
        if dist <= 0.0:
            return True
        start = self._base_in_odom(timeout_sec=1.0)
        if start is None:
            log.warn('[creep] no odom pose -- skipping creep')
            return False
        twist = Twist()
        twist.linear.x = speed
        deadline = time.time() + timeout_sec
        moved, i = 0.0, 0
        while time.time() < deadline:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.02)
            i += 1
            if i % 10:                    # poll odom ~5 Hz, publish at 50 Hz
                continue
            now = self._base_in_odom()
            if now is not None:
                moved = math.hypot(now[0] - start[0], now[1] - start[1])
                if moved >= dist:
                    break
        self._stop_base()
        time.sleep(0.4)          # let the skid-steer base settle before the arm moves
        final = self._base_in_odom(timeout_sec=1.0)
        if final is not None:
            moved = math.hypot(final[0] - start[0], final[1] - start[1])
        log.info(f'[creep] moved {moved:.3f} m of {dist:.3f} m requested')
        return moved >= dist * 0.9

    def approach_column(self, col_xy, color, timeout_sec=60.0, height=None):
        """Front-camera visual servo that centres the base on the `color` column."""
        log = self.get_logger()
        stop_x = self._stop_x_for(height)
        log.info(f'=== PLACE APPROACH: column (front-cam, base-only, {color}) ===')
        if col_xy is not None:
            self._face_box(col_xy)
        deadline = time.time() + timeout_sec
        twist = Twist()
        lost = 0
        while time.time() < deadline:
            det = self.detect_box_front(timeout_sec=0.25, color=color,
                                        gate=self.COLUMN_DETECT_GATE)
            if det is not None:
                lost = 0
                bx, by, _ = det
                if bx <= stop_x and abs(by) <= COLUMN_Y_TOL:
                    self._stop_base()
                    log.info(f'[place] column centred (front {bx:.2f},{by:+.2f})')
                    return True
                fwd = max(0.0, bx - stop_x)
                twist.linear.x = (min(APPROACH_LINEAR_MAX,
                                      max(APPROACH_LINEAR_MIN,
                                          APPROACH_LINEAR_GAIN * fwd))
                                  if fwd > 0.0 else 0.0)
                twist.angular.z = max(-APPROACH_ANGULAR_MAX,
                                      min(APPROACH_ANGULAR_MAX,
                                          APPROACH_ANGULAR_GAIN * math.atan2(by, bx)))
            else:
                lost += 1
                if lost > 12:
                    self._stop_base()
                    log.warn('[place] lost the column -- aborting approach')
                    return False
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            for _ in range(4):
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.03)
        self._stop_base()
        log.warn('[place] column approach timed out')
        return False

    # --- placement ----------------------------------------------------------
    def place_on_column(self, tag_id, height, col_xy, color):
        """Centre the base on the `color` column, then lower the held box onto it."""
        log = self.get_logger()
        if not self.approach_column(col_xy, color, height=height):
            log.warn(f'[place] failed to centre on column {tag_id}')
            return False
        time.sleep(0.4)
        det = self.detect_box_front(timeout_sec=1.5, color=color)
        if det is None:
            log.warn(f'[place] lost sight of column {tag_id} after settling')
            return False
        if self.COLUMN_DEPTH_BIAS > 0.0:
            true_x = det[0] + self.COLUMN_DEPTH_BIAS
            creep = true_x - self.COLUMN_PLACE_X
            log.info(f'[place] column read {det[0]:.2f} m -> true ~{true_x:.2f} m; '
                     f'creeping {creep:.2f} m to place at {self.COLUMN_PLACE_X:.2f}')
            if not self._creep_forward(creep):
                log.warn(f'[place] creep to column {tag_id} fell short -- aborting '
                         f'placement (box still held)')
                return False
            px = self.COLUMN_PLACE_X
            py = det[1]
        else:
            px = min(MAX_REACH_X, det[0] + self.COLUMN_X_OFFSET)
            py = det[1]
        top_z = GROUND_Z + height + self.PAYLOAD_GRIP_HEIGHT
        ceiling = (self.OVER_Z_CEILING_TALL if height >= self.TALL_COLUMN_H
                   else self.OVER_Z_CEILING)
        over_z = min(top_z + BOX_SIZE / 2.0 + 0.04, ceiling)
        log.info(f'=== PLACE: box onto column {tag_id} at ({px:.2f},{py:+.2f}) h={height} ===')
        if not self.move_pose(px, py, over_z, 0.0, label='over-column',
                              quat_xyzw=zdown_quat(0.0), strict=True):
            log.warn(f'[place] over-column move failed for column {tag_id} -- '
                     f'aborting placement (box still held)')
            return False
        if not self.move_pose(px, py, top_z, 0.0, cartesian=True,
                              label='lower-onto-column', quat_xyzw=zdown_quat(0.0)):
            log.warn(f'[place] lower-onto-column move failed for column {tag_id} '
                     f'-- aborting placement (box still held)')
            return False
        self.arm.detach_collision_object(BOX_ID)
        time.sleep(0.3)
        self.gripper(GRIP_OPEN, 'release')
        self.move_pose(px, py, over_z, 0.0, cartesian=True, label='retreat',
                       quat_xyzw=zdown_quat(0.0))
        self.arm.remove_collision_object(BOX_ID)
        log.info('[place] backing off the column')
        self._drive_blind(-0.22, 3.5)
        self._stop_base()
        self.move_config(HOME_CONFIG, 'gripper-down ready')
        if not self._placement_landed(height, color, tag_id):
            return False
        log.info(f'=== PLACE: DONE (column {tag_id}) ===')
        return True

    def _placement_landed(self, height, color, tag_id):
        """True if the box ended up on the column rather than on the floor."""
        log = self.get_logger()
        mark_z = height + self.PAYLOAD_GRIP_HEIGHT
        lo = max(height + 0.005, mark_z - BOX_SIZE) - FRONT_CAM_Z
        hi = mark_z + BOX_SIZE - FRONT_CAM_Z
        gate = (0.05, 1.5, -0.6, 0.6, lo, hi)
        det = self.detect_box_front(timeout_sec=2.0, color=color, gate=gate)
        if det is None:
            log.error(f'[place] no {color} box above column {tag_id}\'s top '
                      f'after release -- it is not on the column. FAILED.')
            return False
        bz = det[2]
        floor_z = self.PAYLOAD_FLOOR_Z
        placed_z = self.PAYLOAD_FLOOR_Z + height
        if bz < (floor_z + placed_z) / 2.0:
            log.error(f'[place] {color} box is at z={bz:.3f} -- that is floor '
                      f'level (~{floor_z:.3f}), not column {tag_id} top '
                      f'(~{placed_z:.3f}). Placement FAILED.')
            return False
        log.info(f'[place] verified: {color} box at z={bz:.3f} '
                 f'(column top ~{placed_z:.3f})')
        return True

    # --- full mission -------------------------------------------------------
    def run_mission_2(self):
        log = self.get_logger()
        log.info('=== MISSION 2: START ===')
        if not self.wait_for_localization():
            return

        for (color, box_xy), (tag_id, height, col_xy) in zip(self.LAYOUT_BOXES,
                                                             self.LAYOUT_COLUMNS):
            log.info(f'--- {color} box -> column {tag_id} (h={height}) ---')

            self._set_yaw_goal_tolerance(TIGHT_YAW_TOLERANCE)
            nav_ok = self.navigate_to(self.make_map_goal(*self.LAYOUT_TABLE_APPROACH))
            self._set_yaw_goal_tolerance(DEFAULT_YAW_TOLERANCE)
            if not nav_ok:
                log.error('Table navigation failed -- aborting.')
                return
            box_map = (box_xy[0], box_xy[1])
            if not self.claw_pick(box_map, color=color,
                                  grasp_z=self.LAYOUT_TABLE_GRASP_Z,
                                  x_offset=self.LAYOUT_TABLE_X_OFFSET):
                log.error(f'Failed to pick the {color} box -- aborting.')
                return

            log.info('[mission2] backing off the table')
            self._drive_blind(-0.18, 3.5)
            self._stop_base()

            adx, ady = self.PLACE_APPROACH_DIR
            approach = (col_xy[0] + adx * NAV_STANDOFF,
                        col_xy[1] + ady * NAV_STANDOFF,
                        math.atan2(-ady, -adx))
            self._set_yaw_goal_tolerance(TIGHT_YAW_TOLERANCE)
            nav_ok = self.navigate_to(self.make_map_goal(*approach))
            self._set_yaw_goal_tolerance(DEFAULT_YAW_TOLERANCE)
            if not nav_ok:
                log.error(f'Column {tag_id} navigation failed -- aborting.')
                return
            if not self.grasp_is_holding():
                log.error(f'{color} box was DROPPED during the carry to column '
                          f'{tag_id} (gripper is empty on arrival) -- aborting.')
                return
            if not self.place_on_column(tag_id, height, col_xy, color):
                log.error(f'Failed to place on column {tag_id} -- aborting.')
                return

        log.info('=== MISSION 2: parking ===')
        self.navigate_to(self.make_map_goal(*self.LAYOUT_FINAL_POSE))
        log.info('=== MISSION 2: DONE ===')


class Mission2Tugbot(Mission2):
    """Mission 2 in the OpenRobotics Tugbot warehouse world."""
    TUGBOT_GATE = (0.05, 2.5, -0.7, 0.7, -0.25, 0.36)
    COLUMN_DETECT_GATE = TUGBOT_GATE

    def detect_box_front(self, timeout_sec=2.0, debug_save=False, color='blue', gate=None):
        if gate is None:
            gate = self.TUGBOT_GATE
        return super().detect_box_front(timeout_sec, debug_save, color, gate)


class Mission2Hospital(Mission2):
    """Mission 2 in the AWS hospital, carrying racks."""
    # --- what is carried ------------------------------------------------------
    GRASP_MODELS = {c: f'rack_{c}' for c in BOX_COLORS}
    PAYLOAD_GRIP_HEIGHT = HOSP_RACK_GRIP_HEIGHT
    PAYLOAD_FLOOR_Z = GROUND_Z + HOSP_RACK_GRIP_HEIGHT

    # --- where it happens -----------------------------------------------------
    LAYOUT_TABLE_APPROACH = HOSP_TABLE_APPROACH
    LAYOUT_TABLE_GRASP_Z = GROUND_Z + HOSP_TABLE_TOP + HOSP_RACK_GRIP_HEIGHT
    LAYOUT_TABLE_X_OFFSET = TABLE_X_OFFSET
    LAYOUT_BOXES = HOSP_RACKS
    LAYOUT_COLUMNS = HOSP_COLUMNS
    LAYOUT_FINAL_POSE = HOSP_FINAL_POSE
    PLACE_APPROACH_DIR = HOSP_PLACE_APPROACH_DIR

    # --- rejecting the building -----------------------------------------------
    HOSPITAL_GATE = (0.05, 2.5, -0.7, 0.7, -0.25, 0.32)
    COLUMN_DETECT_GATE = HOSPITAL_GATE

    def detect_box_front(self, timeout_sec=2.0, debug_save=False, color='blue',
                         gate=None):
        if gate is None:
            gate = self.HOSPITAL_GATE
        return super().detect_box_front(timeout_sec, debug_save, color, gate)


def _run(node_cls):
    rclpy.init()
    node = node_cls()
    ex = rclpy.executors.MultiThreadedExecutor(4)
    ex.add_node(node)

    def task():
        time.sleep(3.0)
        node.run_mission_2()

    t = threading.Thread(target=task, daemon=True)
    t.start()
    try:
        while rclpy.ok():
            try:
                ex.spin()
                break
            except KeyboardInterrupt:
                raise
            except Exception as exc:
                node.get_logger().warn(f'[executor] recovered from spin error: {exc}')
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


def main_tugbot():
    _run(Mission2Tugbot)


def main_hospital():
    _run(Mission2Hospital)


if __name__ == '__main__':
    main()
