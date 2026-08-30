#!/usr/bin/env python3
"""Autonomous search-and-pick: drive the base until the wrist camera finds the box."""
import math
import time
import threading

import rclpy
from geometry_msgs.msg import Twist

from pickplace_arm_bringup.pick_and_place import (
    PickAndPlace, scan_quat)

# --- unified search/approach vantage point (base_link frame) ------------------
SEARCH_POSITION = (0.45, 0.00, 0.30)
SEARCH_PITCH = math.radians(143.6)
GRASP_SCAN_POSITION = (0.30, 0.00, 0.42)
GRASP_SCAN_PITCH = math.radians(66.6)

# --- search state machine tuning ------------------------------------------------
SPIN_ANGULAR = 0.5              # rad/s during each rotation step
SPIN_STEP_RAD = math.radians(12.0)   # heading increment per scan
SPIN_SETTLE_SEC = 0.6          # let motion damp out before capturing
SPIN_STEPS_PER_REV = int(round(2 * math.pi / SPIN_STEP_RAD)) + 1
BLIND_FORWARD_LINEAR = 0.2      # m/s, when a full spin finds nothing
BLIND_FORWARD_SEC = 2.0

APPROACH_LINEAR_GAIN = 1.0
APPROACH_LINEAR_MAX = 0.60      # m/s
APPROACH_LINEAR_MIN = 0.12
APPROACH_ANGULAR_GAIN = 1.4
APPROACH_ANGULAR_MAX = 0.9      # rad/s
STOP_DISTANCE = 0.46            # m; must stay >= the search pose's ~0.45m

SEARCH_TIMEOUT_SEC = 180.0


class SearchAndPick(PickAndPlace):
    def __init__(self):
        super().__init__()
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'diff_drive_controller/cmd_vel_unstamped', 10)
        self.scan_position = GRASP_SCAN_POSITION
        self.scan_pitch = GRASP_SCAN_PITCH
        self.get_logger().info('Search-and-pick node ready')

    def _stop_base(self):
        self.cmd_vel_pub.publish(Twist())

    def _drive_blind(self, linear_x, duration_sec):
        twist = Twist()
        twist.linear.x = linear_x
        end = time.time() + duration_sec
        while time.time() < end:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.1)
        self._stop_base()

    def _rotate_step(self, angle_rad):
        """Rotate in place by ~angle_rad, then stop and let the base settle."""
        twist = Twist()
        twist.angular.z = SPIN_ANGULAR if angle_rad >= 0 else -SPIN_ANGULAR
        end = time.time() + abs(angle_rad) / SPIN_ANGULAR
        while time.time() < end:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self._stop_base()
        time.sleep(SPIN_SETTLE_SEC)

    def search_and_approach(self, timeout_sec=SEARCH_TIMEOUT_SEC):
        log = self.get_logger()
        log.info('=== SEARCH: START ===')

        sx, sy, sz = SEARCH_POSITION
        self.move_pose(sx, sy, sz, label='search-scan',
                       quat_xyzw=scan_quat(SEARCH_PITCH))

        twist = Twist()
        deadline = time.time() + timeout_sec
        steps_since_detection = 0

        while time.time() < deadline:
            detection = self.detect_box_pose(timeout_sec=1.0)

            if detection is not None:
                steps_since_detection = 0
                bx, by, _bz = detection
                dist = math.hypot(bx, by)
                bearing = math.atan2(by, bx)
                log.info(f'[search] box seen: dist={dist:.2f}m '
                          f'bearing={math.degrees(bearing):.1f}deg')

                if dist < STOP_DISTANCE:
                    self._stop_base()
                    log.info('=== SEARCH: box within reach, stopping ===')
                    return True

                nudge_sec = 0.3
                twist.linear.x = min(APPROACH_LINEAR_MAX,
                                      APPROACH_LINEAR_GAIN * dist)
                twist.angular.z = max(-APPROACH_ANGULAR_MAX, min(
                    APPROACH_ANGULAR_MAX, APPROACH_ANGULAR_GAIN * bearing))
                end = time.time() + nudge_sec
                while time.time() < end:
                    self.cmd_vel_pub.publish(twist)
                    time.sleep(0.05)
                self._stop_base()
                time.sleep(SPIN_SETTLE_SEC)
            else:
                self._rotate_step(SPIN_STEP_RAD)
                steps_since_detection += 1
                if steps_since_detection >= SPIN_STEPS_PER_REV:
                    log.info('[search] full rotation, nothing found -- '
                              'driving forward and retrying')
                    self._drive_blind(BLIND_FORWARD_LINEAR, BLIND_FORWARD_SEC)
                    time.sleep(SPIN_SETTLE_SEC)
                    steps_since_detection = 0

        self._stop_base()
        log.error('=== SEARCH: timed out without finding the box ===')
        return False


def main():
    rclpy.init()
    node = SearchAndPick()
    ex = rclpy.executors.MultiThreadedExecutor(4)
    ex.add_node(node)

    def task():
        time.sleep(3.0)
        if node.search_and_approach():
            node.run()
        else:
            node.get_logger().error('Search failed -- not attempting pick.')

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
