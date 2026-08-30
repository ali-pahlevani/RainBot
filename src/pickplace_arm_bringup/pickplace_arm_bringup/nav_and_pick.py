#!/usr/bin/env python3
"""Autonomous navigate-and-pick: search the map, approach the box, hand off to the pick."""
import math
import time
import threading

import rclpy
from rclpy.duration import Duration as RclDuration
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from action_msgs.msg import GoalStatus
import tf2_ros
import tf2_geometry_msgs  # registers do_transform_point for PointStamped

from nav2_msgs.action import NavigateToPose

from pickplace_arm_bringup.pick_and_place import scan_quat
from pickplace_arm_bringup.search_and_pick import (
    SearchAndPick, SEARCH_POSITION, SEARCH_PITCH, SPIN_ANGULAR, SPIN_STEP_RAD,
    SPIN_SETTLE_SEC, SPIN_STEPS_PER_REV)

APPROACH_DIST = 1.4
NAV_TIMEOUT_SEC = 120.0

EXPLORE_WAYPOINTS = [
    (1.8, 0.0), (1.3, 1.3), (0.0, 1.8), (-1.3, 1.3),
    (-1.8, 0.0), (-1.3, -1.3), (0.0, -1.8), (1.3, -1.3),
]


class NavAndPick(SearchAndPick):
    def __init__(self):
        super().__init__()
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info('Nav-and-pick node ready')

    # --- scan one full revolution in place, return the box if seen ----------
    def scan_in_place(self):
        log = self.get_logger()
        for _ in range(SPIN_STEPS_PER_REV):
            det = self.detect_box_pose(timeout_sec=1.0)
            if det is not None:
                bx, by, _ = det
                log.info(f'[explore] box seen: dist={math.hypot(bx,by):.2f}m '
                          f'bearing={math.degrees(math.atan2(by,bx)):.1f}deg')
                return det
            self._rotate_step(SPIN_STEP_RAD)
        return self.detect_box_pose(timeout_sec=1.0)

    # --- coverage search: scan at start, then at Nav2-reached waypoints ------
    def explore_and_find(self):
        log = self.get_logger()
        sx, sy, sz = SEARCH_POSITION
        self.move_pose(sx, sy, sz, label='search-scan',
                       quat_xyzw=scan_quat(SEARCH_PITCH))

        det = self.scan_in_place()
        if det is not None:
            return det

        for i, (wx, wy) in enumerate(EXPLORE_WAYPOINTS):
            log.info(f'[explore] -> waypoint {i + 1}/{len(EXPLORE_WAYPOINTS)} '
                      f'map({wx:.1f},{wy:.1f})')
            goal = self.make_map_goal(wx, wy, math.atan2(wy, wx))
            if not self.navigate_to(goal, timeout_sec=60.0):
                log.warn(f'[explore] could not reach waypoint {i + 1} -- skipping')
                continue
            det = self.scan_in_place()
            if det is not None:
                return det

        log.error('[explore] box not found at any waypoint')
        return None

    # --- transform a base_link point into the map frame ---------------------
    def box_in_map(self, bx, by):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', self.tf_frame('base_link'), rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'[nav] map<-base_link TF failed: {e}')
            return None
        pt = PointStamped()
        pt.header.frame_id = self.tf_frame('base_link')
        pt.point.x, pt.point.y, pt.point.z = bx, by, 0.0
        pm = tf2_geometry_msgs.do_transform_point(pt, tf)
        return pm.point.x, pm.point.y

    def robot_in_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', self.tf_frame('base_link'), rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f'[nav] robot pose TF failed: {e}')
            return None
        t = tf.transform.translation
        return t.x, t.y

    def make_map_goal(self, mx, my, yaw):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = mx
        goal.pose.position.y = my
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)
        return goal

    def compute_approach_goal(self, box_map, robot_map):
        bx, by = box_map
        rx, ry = robot_map
        dx, dy = bx - rx, by - ry
        d = math.hypot(dx, dy)
        ux, uy = (1.0, 0.0) if d < 1e-3 else (dx / d, dy / d)
        # stop APPROACH_DIST short of the box, facing it
        return self.make_map_goal(bx - APPROACH_DIST * ux,
                                  by - APPROACH_DIST * uy, math.atan2(uy, ux))

    def navigate_to(self, goal_pose, timeout_sec=NAV_TIMEOUT_SEC, retries=2):
        """Send a NavigateToPose goal and wait for it to succeed."""
        log = self.get_logger()
        if not self.nav_client.wait_for_server(timeout_sec=45.0):
            log.error('[nav] NavigateToPose action server unavailable')
            return False
        for attempt in range(retries + 1):
            goal_msg = NavigateToPose.Goal()
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose = goal_pose
            log.info(f'[nav] sending Nav2 goal '
                      f'({goal_pose.pose.position.x:.2f},'
                      f'{goal_pose.pose.position.y:.2f})'
                      f'{" (retry)" if attempt else ""}')
            send_fut = self.nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_fut, timeout_sec=10.0)
            handle = send_fut.result()
            if handle is None or not handle.accepted:
                log.warn('[nav] Nav2 goal rejected -- retrying')
                time.sleep(1.5)
                continue
            result_fut = handle.get_result_async()
            deadline = self.get_clock().now() + RclDuration(seconds=timeout_sec)
            wall_stop = time.time() + timeout_sec * 4.0
            while rclpy.ok() and not result_fut.done():
                rclpy.spin_until_future_complete(self, result_fut,
                                                 timeout_sec=0.5)
                if self.get_clock().now() >= deadline:
                    log.warn(f'[nav] {timeout_sec:.0f}s of sim time elapsed')
                    break
                if time.time() > wall_stop:
                    log.warn('[nav] wall-clock backstop hit -- is /clock '
                             'running?')
                    break
            res = result_fut.result()
            if res is None:
                log.error('[nav] Nav2 goal timed out')
                return False
            if res.status == GoalStatus.STATUS_SUCCEEDED:
                log.info('[nav] Nav2 reached goal')
                return True
            log.warn(f'[nav] Nav2 goal did not succeed (status={res.status}) '
                      f'-- settling and retrying')
            self._stop_base()
            time.sleep(2.0)
        log.error('[nav] Nav2 goal failed after retries')
        return False

    def run_autonomous(self):
        log = self.get_logger()
        log.info('=== NAV AND PICK: START ===')

        det = self.explore_and_find()
        if det is None:
            log.error('Box never detected -- aborting.')
            return
        bx, by, _ = det

        box_map = self.box_in_map(bx, by)
        robot_map = self.robot_in_map()
        if box_map is None or robot_map is None:
            log.error('TF unavailable -- cannot build Nav2 goal.')
            return
        log.info(f'[nav] box in map: ({box_map[0]:.2f},{box_map[1]:.2f})')

        goal = self.compute_approach_goal(box_map, robot_map)
        if not self.navigate_to(goal):
            log.error('Navigation failed -- aborting pick.')
            return

        if self.search_and_approach():
            self.run()
        else:
            log.error('Visual approach failed after navigation -- no pick.')


def main():
    rclpy.init()
    node = NavAndPick()
    ex = rclpy.executors.MultiThreadedExecutor(4)
    ex.add_node(node)

    def task():
        time.sleep(3.0)
        node.run_autonomous()

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
