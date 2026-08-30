#!/usr/bin/env python3
"""Full autonomous warehouse mission: search, approach, pick, deliver, park."""
import math
import time
import threading

import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
import tf2_ros

from nav2_msgs.action import NavigateThroughPoses

from pickplace_arm_bringup.nav_and_pick import NavAndPick, APPROACH_DIST
from pickplace_arm_bringup.pick_and_place import (
    HOME_CONFIG, scan_quat, GRIPPER_Y)
from pickplace_arm_bringup.search_and_pick import (
    APPROACH_LINEAR_GAIN, APPROACH_LINEAR_MAX, APPROACH_LINEAR_MIN,
    APPROACH_ANGULAR_GAIN, APPROACH_ANGULAR_MAX, SEARCH_POSITION, SEARCH_PITCH,
    GRASP_SCAN_POSITION, GRASP_SCAN_PITCH)

PHASE2_HANDOFF_DIST = 0.60
STOP_DISTANCE_FINE = 0.41

CLAW_STOP_X = 0.75
CLAW_Y_TOL = 0.02

FRONT_HANDOFF_DIST = 0.9

# --- mission targets (map frame; map origin = robot's mapping start pose) -----
PATROL_WAYPOINTS = [
    (2.5, 0.0), (2.5, -3.5), (-1.0, -4.0), (-3.5, -1.5),
    (-3.5, 1.5), (0.0, 2.0), (2.0, 2.0),
]
DELIVERY_POSE = (-4.0, 2.0, 0.0)     # x, y, yaw -- where the box is dropped off
PARKING_POSE = (4.0, -4.0, 0.0)      # x, y, yaw -- final parking station

FRONT_DETECT_MAX_DIST = 2.5   # only act on front-cam detections within this range
FRONT_DETECT_CONSEC = 2       # consecutive detections before committing
SEARCH_TIMEOUT_SEC = 240.0


class Mission(NavAndPick):
    def __init__(self):
        super().__init__()
        self.tp_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')
        self.get_logger().info('Mission node ready')

    # --- helpers ------------------------------------------------------------
    def _patrol_poses(self, reverse=False):
        wps = list(reversed(PATROL_WAYPOINTS)) if reverse else list(PATROL_WAYPOINTS)
        poses = []
        prev_yaw = 0.0
        for i, (x, y) in enumerate(wps):
            # face the next leg; the final pose keeps the previous heading
            if i + 1 < len(wps):
                nx, ny = wps[i + 1]
                prev_yaw = math.atan2(ny - y, nx - x)
            poses.append(self.make_map_goal(x, y, prev_yaw))
        return poses

    def wait_for_localization(self, timeout_sec=60.0):
        log = self.get_logger()
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            try:
                self.tf_buffer.lookup_transform(
                    'map', self.tf_frame('base_link'), rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=1.0))
                log.info('[mission] localized (map->base_link available)')
                return True
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException):
                time.sleep(0.5)
        log.error('[mission] no map->base_link TF -- is AMCL running?')
        return False

    def _cancel(self, handle):
        try:
            fut = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        except Exception:
            pass
        self._stop_base()

    # --- SEARCH: patrol (no stopping) while the front camera watches ---------
    def search_via_patrol(self, timeout_sec=SEARCH_TIMEOUT_SEC):
        log = self.get_logger()
        log.info('=== MISSION SEARCH: patrol + front-camera watch ===')
        self.move_config(HOME_CONFIG, 'home')

        if not self.tp_client.wait_for_server(timeout_sec=10.0):
            log.error('[mission] NavigateThroughPoses server unavailable')
            return None

        def send_patrol(reverse):
            g = NavigateThroughPoses.Goal()
            g.poses = self._patrol_poses(reverse)
            fut = self.tp_client.send_goal_async(g)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            h = fut.result()
            if h is None or not h.accepted:
                return None, None
            return h, h.get_result_async()

        reverse = False
        handle, result_fut = send_patrol(reverse)
        if handle is None:
            log.error('[mission] patrol goal rejected')
            return None

        consec = 0
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            det = self.detect_box_front(timeout_sec=1.0)
            if det is not None and math.hypot(det[0], det[1]) < FRONT_DETECT_MAX_DIST:
                consec += 1
                log.info(f'[mission] front camera sees box: '
                          f'dist={math.hypot(det[0], det[1]):.2f}m ({consec})')
                if consec >= FRONT_DETECT_CONSEC:
                    log.info('=== MISSION SEARCH: box found, stopping patrol ===')
                    self._cancel(handle)
                    return det
            else:
                consec = 0
            if result_fut.done():   # lap finished without finding -> reverse + re-patrol
                log.info('[mission] patrol lap done, no box -- reversing route')
                reverse = not reverse
                handle, result_fut = send_patrol(reverse)
                if handle is None:
                    return None
                time.sleep(1.0)     # let the new lap start driving before re-checking
        self._cancel(handle)
        log.error('=== MISSION SEARCH: timed out ===')
        return None

    # --- APPROACH: front-cam coarse drive-in, then wrist fine servo ----------
    def _drive_toward(self, dist, bearing, stop_slack):
        """One proportional nudge toward a box at (dist, bearing), then stop and settle."""
        margin = max(0.0, dist - stop_slack)
        twist = Twist()
        twist.linear.x = min(APPROACH_LINEAR_MAX,
                             max(APPROACH_LINEAR_MIN, APPROACH_LINEAR_GAIN * margin))
        twist.angular.z = max(-APPROACH_ANGULAR_MAX,
                              min(APPROACH_ANGULAR_MAX, APPROACH_ANGULAR_GAIN * bearing))
        burst = min(0.6, max(0.25, 0.9 * margin))
        end = time.time() + burst
        while time.time() < end:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self._stop_base()
        time.sleep(0.15)

    def _servo_phase(self, detect, stop_dist, stop_slack, sweep_cap, deadline):
        """Servo the base toward the box using `detect` until it is within stop_dist."""
        log = self.get_logger()
        sweep = 0.0
        going_left = True
        while time.time() < deadline:
            det = detect(timeout_sec=1.0)
            if det is not None:
                sweep = 0.0
                bx, by, _ = det
                dist = math.hypot(bx, by)
                bearing = math.atan2(by, bx)
                log.info(f'[approach] box: dist={dist:.2f}m '
                          f'bearing={math.degrees(bearing):.0f}deg')
                if dist < stop_dist:
                    self._stop_base()
                    return 'reached'
                self._drive_toward(dist, bearing, stop_slack)
            else:
                step = 0.15 if going_left else -0.15
                self._rotate_step(step)
                sweep += step
                if going_left and sweep >= sweep_cap:
                    going_left = False
                elif not going_left and sweep <= -sweep_cap:
                    self._stop_base()
                    return 'lost'
        return 'timeout'

    def _face_box(self, box_map):
        """Turn in place to face the box's known map position, as one bounded open-loop turn."""
        log = self.get_logger()
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', self.tf_frame('base_link'), rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return
        t = tf.transform.translation
        q = tf.transform.rotation
        ryaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        bearing = math.atan2(box_map[1] - t.y, box_map[0] - t.x)
        err = math.atan2(math.sin(bearing - ryaw), math.cos(bearing - ryaw))
        if abs(err) < 0.12:
            return
        err = max(-1.0, min(1.0, err))   # cap the correction at ~57 deg
        log.info(f'[approach] facing box (turn {math.degrees(err):.0f} deg)')
        self._rotate_step(err)

    def servo_to_box(self, box_map=None, timeout_sec=70.0):
        """Final approach after Nav2: front camera, then wrist search pose, then grasp scan."""
        log = self.get_logger()
        log.info('=== MISSION APPROACH: front-cam coarse + wrist fine ===')
        deadline = time.time() + timeout_sec

        if box_map is not None and face_first:
            self._face_box(box_map)

        r = self._servo_phase(self.detect_box_front, FRONT_HANDOFF_DIST,
                               FRONT_HANDOFF_DIST - 0.3, sweep_cap=0.35,
                               deadline=deadline)
        if r != 'reached':
            log.warn(f'[approach] front-cam phase {r} -- aborting approach')
            return False

        self.move_pose(*SEARCH_POSITION, label='search-scan',
                       quat_xyzw=scan_quat(SEARCH_PITCH))
        r = self._servo_phase(self.detect_box_pose, PHASE2_HANDOFF_DIST,
                              PHASE2_HANDOFF_DIST - 0.15, sweep_cap=0.5,
                              deadline=deadline)
        if r != 'reached':
            log.warn(f'[approach] wrist phase {r} -- aborting approach')
            return False

        self.move_pose(*GRASP_SCAN_POSITION, label='grasp-scan',
                       quat_xyzw=scan_quat(GRASP_SCAN_PITCH))
        r = self._servo_phase(self.detect_box_pose, STOP_DISTANCE_FINE,
                              STOP_DISTANCE_FINE - 0.10, sweep_cap=0.4,
                              deadline=deadline)
        if r == 'reached':
            log.info('=== MISSION APPROACH: box within grasp reach ===')
            return True
        log.warn(f'[approach] fine phase {r} -- aborting approach')
        return False

    # --- full mission -------------------------------------------------------
    # --- claw approach: keep the gripper down, drive the box under it ---------
    def claw_approach(self, box_map, timeout_sec=60.0, color='blue',
                      face_first=True):
        """Drive the base with the front camera until the `color` box is CLAW_STOP_X ahead."""
        log = self.get_logger()
        log.info('=== MISSION APPROACH: claw (gripper-down, continuous) ===')
        self.move_config(HOME_CONFIG, 'gripper-down ready')
        if box_map is not None:
            self._face_box(box_map)
        deadline = time.time() + timeout_sec
        twist = Twist()
        lost = 0
        while time.time() < deadline:
            det = self.detect_box_front(timeout_sec=0.25, color=color)
            if det is not None:
                lost = 0
                bx, by, _ = det
                if bx <= CLAW_STOP_X and abs(by) <= CLAW_Y_TOL:
                    self._stop_base()
                    log.info(f'[claw] box within reach (front {bx:.2f},{by:+.2f})')
                    return True
                fwd = max(0.0, bx - CLAW_STOP_X)
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
                    log.warn('[claw] lost the box -- aborting approach')
                    return False
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            for _ in range(3):
                self.cmd_vel_pub.publish(twist)
                time.sleep(0.03)
        self._stop_base()
        log.warn('[claw] approach timed out')
        return False

    def claw_pick(self, box_map, color='blue', grasp_z=None, x_offset=None,
                  face_first=True):
        """Drive the `color` box under the gripper and descend onto it, retrying a few times."""
        from pickplace_arm_bringup.pick_and_place import GRASP_Z, FRONT_X_OFFSET
        if grasp_z is None:
            grasp_z = GRASP_Z
        if x_offset is None:
            x_offset = FRONT_X_OFFSET
        log = self.get_logger()
        for attempt in range(1, 4):
            log.info(f'--- claw pick attempt {attempt}/3 ({color}) ---')
            if not self.claw_approach(box_map, color=color,
                                      face_first=face_first):
                return False
            if self.grab_below(grasp_z=grasp_z, color=color, x_offset=x_offset):
                return True
            log.warn('[claw] grab missed -- re-centring and retrying')
            face_first = True
            self.move_config(HOME_CONFIG, 'gripper-down ready')
            self._drive_blind(-0.15, 2.0)
        return False

    def run_mission(self):
        log = self.get_logger()
        log.info('=== MISSION: START ===')

        if not self.wait_for_localization():
            return

        det = self.search_via_patrol()
        if det is None:
            log.error('Box never found -- mission aborted.')
            return

        self._stop_base()
        time.sleep(2.0)
        fresh = self.detect_box_front(timeout_sec=2.0)
        if fresh is not None:
            det = fresh
        bx, by, _ = det

        # APPROACH: box -> map, Nav2 to ~APPROACH_DIST in front, then wrist servo
        box_map = self.box_in_map(bx, by)
        robot_map = self.robot_in_map()
        if box_map is None or robot_map is None:
            log.error('TF unavailable for approach goal -- aborting.')
            return
        log.info(f'[mission] box in map: ({box_map[0]:.2f},{box_map[1]:.2f})')
        if not self.navigate_to(self.compute_approach_goal(box_map, robot_map)):
            log.error('Approach navigation failed -- aborting.')
            return

        if not self.claw_pick(box_map):
            log.error('Claw pick failed -- aborting.')
            return

        # DELIVER (carry the box to the delivery point)
        log.info('=== MISSION: delivering to drop-off ===')
        if not self.navigate_to(self.make_map_goal(*DELIVERY_POSE)):
            log.error('Delivery navigation failed -- placing where we are.')
        self.place_box_down()

        # PARK
        log.info('=== MISSION: parking ===')
        self.navigate_to(self.make_map_goal(*PARKING_POSE))
        log.info('=== MISSION: DONE ===')


def main():
    rclpy.init()
    node = Mission()
    ex = rclpy.executors.MultiThreadedExecutor(4)
    ex.add_node(node)

    def task():
        time.sleep(3.0)
        node.run_mission()

    t = threading.Thread(target=task, daemon=True)
    t.start()
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
