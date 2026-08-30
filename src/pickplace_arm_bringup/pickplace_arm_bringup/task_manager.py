#!/usr/bin/env python3
"""Assigns errands, hands out delivery slots and parking vertices, and draws the job in RViz."""
import math
import os
import sys
import threading
import time

import rclpy
import tf2_ros
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from pickplace_arm_bringup.fleet_layout import ROBOTS, parking_vertices
from pickplace_arm_bringup import rack_table_layout as RT

DEFAULT_DEPART_STAGGER = float(os.environ.get('FLEET_DEPART_STAGGER', 20.0))

COLOUR_RGBA = {'red': (0.9, 0.05, 0.05, 1.0),
               'green': (0.05, 0.7, 0.05, 1.0),
               'blue': (0.1, 0.5, 0.9, 1.0)}


class TaskManager(Node):
    def __init__(self):
        super().__init__('task_manager')
        self.declare_parameter('depart_stagger', DEFAULT_DEPART_STAGGER)
        self.depart_stagger = float(
            self.get_parameter('depart_stagger').value)
        self.robots = [ns for ns, *_ in ROBOTS]
        self.collections = RT.collection_points()
        self.slots = RT.delivery_slots()
        self.parks = parking_vertices()

        self._lock = threading.Lock()
        self._free_slots = list(range(len(self.slots)))
        self._free_parks = list(range(len(self.parks)))
        self._slot_owner = {}
        self._park_owner = {}
        self._delivery_busy = None          # ns currently using the table
        self.status = {ns: ('waiting', '') for ns in self.robots}
        self.delivered = set()
        self.holding = set()
        self.finished = set()
        # Placing order: r1, then r2, then r3 -- fleet order, not arrival order.
        self.place_order = list(self.robots)
        self.assignment = {}

        task_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE,
                              history=HistoryPolicy.KEEP_LAST)
        self.task_pubs = {ns: self.create_publisher(String, f'/{ns}/task', task_qos)
                          for ns in self.robots}
        for ns in self.robots:
            self.create_subscription(
                String, f'/{ns}/mission_status',
                lambda m, k=ns: self._on_status(k, m), 10)
            self.create_service(Trigger, f'/{ns}/claim_slot',
                                lambda req, res, k=ns: self._claim_slot(k, res))
            self.create_service(Trigger, f'/{ns}/claim_park',
                                lambda req, res, k=ns: self._claim_park(k, res))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(MarkerArray, '/fleet/markers', 1)
        self.board_pub = self.create_publisher(String, '/fleet/status', 10)
        self.create_timer(1.0, self._publish_markers)
        self.create_timer(2.0, self._publish_board)
        self.get_logger().info(
            f'task manager up: {len(self.robots)} robots, '
            f'{len(self.collections)} collection points, {len(self.slots)} slots')

    # --- allocation -----------------------------------------------------------
    def _claim_slot(self, ns, res):
        """Grant the delivery table only when every robot is holding, and then in fleet order."""
        with self._lock:
            self.holding.add(ns)
            pending = [r for r in self.robots
                       if r not in self.holding and r not in self.finished]
            if pending:
                res.success = False
                res.message = (f'in use: waiting for {", ".join(pending)} to '
                               f'reach the holding ring')
                return res
            turn = next((r for r in self.place_order
                         if r not in self.finished), None)
            if turn is not None and turn != ns:
                res.success = False
                res.message = f'in use by {turn} (placing in fleet order)'
                return res
            if self._delivery_busy not in (None, ns):
                res.success = False
                res.message = f'delivery table in use by {self._delivery_busy}'
                return res
            if not self._free_slots:
                res.success = False
                res.message = 'no free slot'
                return res
            slot = self._free_slots.pop(0)
            self._slot_owner[slot] = ns
            self._delivery_busy = ns
            res.success = True
            res.message = str(slot)
            self.get_logger().info(
                f'[manager] {ns} -> delivery slot {slot} '
                f'(table now held by {ns})')
            return res

    def _claim_park(self, ns, res):
        with self._lock:
            if not self._free_parks:
                res.success = False
                res.message = 'no free parking vertex'
                return res
            v = self._free_parks.pop(0)
            self._park_owner[v] = ns
            res.success = True
            res.message = str(v)
            self.get_logger().info(f'[manager] {ns} -> parking vertex {v}')
            return res

    # --- watching the fleet ---------------------------------------------------
    def _on_status(self, ns, msg):
        try:
            _who, state, detail = msg.data.split('|', 2)
        except ValueError:
            return
        self.status[ns] = (state, detail)
        if state == 'delivered':
            self.delivered.add(ns)
        if state == 'holding':
            with self._lock:
                self.holding.add(ns)
        if state in ('done', 'failed'):
            with self._lock:
                self.finished.add(ns)
                self.holding.discard(ns)
                if self._delivery_busy == ns:
                    self._delivery_busy = None
                    self.get_logger().info(f'[manager] {ns} released the delivery table')

    # --- dispatch -------------------------------------------------------------
    def wait_ready(self, ns, timeout_sec=600.0):
        """Is this robot localized, and staying localized?"""
        STABLE_FOR = 6              # 3 s of continuous localization
        deadline = time.time() + timeout_sec
        good = 0
        while time.time() < deadline and rclpy.ok():
            try:
                self.tf_buffer.lookup_transform('map', f'{ns}/base_link',
                                                rclpy.time.Time())
                good += 1
                if good >= STABLE_FOR:
                    return True
            except Exception:
                if good:
                    self.get_logger().info(
                        f'[manager] {ns} localization flickered -- still coming up')
                good = 0
            time.sleep(0.5)
        return False

    def dispatch(self):
        """Give every robot an errand."""
        pairs = list(zip(self.robots, self.collections))
        for ns, (name, colour, table, rack, stand) in pairs:
            self.assignment[ns] = (name, colour)

        pos = {ns: (x, y) for ns, x, y, _ in ROBOTS}
        pairs.sort(key=lambda pr: pos[pr[0]][1])
        self.get_logger().info(
            '[manager] departure order (southernmost first): '
            + ', '.join(p[0] for p in pairs))
        self.get_logger().info('[manager] assignment: ' + ', '.join(
            f'{ns}->{c} at {n}' for ns, (n, c) in self.assignment.items()))

        ready = []
        for ns, spec in pairs:
            if self.wait_ready(ns):
                self.get_logger().info(f'[manager] {ns} is ready')
                ready.append((ns, spec))
            else:
                self.get_logger().error(
                    f'[manager] {ns} never became localized -- no errand for it')
                self.status[ns] = ('failed', 'never localized')

        for i, (ns, (name, colour, table, rack, stand)) in enumerate(ready):
            if i and self.depart_stagger > 0.0:
                self.get_logger().info(
                    f'[manager] holding {ns} for {self.depart_stagger:.0f}s so '
                    f'{ready[i-1][0]} can clear the formation')
                time.sleep(self.depart_stagger)
            msg = String()
            msg.data = '|'.join([
                colour,
                ','.join(f'{v:.6f}' for v in table),
                ','.join(f'{v:.6f}' for v in rack),
                ','.join(f'{v:.6f}' for v in stand)])
            self.task_pubs[ns].publish(msg)
            self.get_logger().info(f'[manager] dispatched {ns} -> {colour} at {name}')

    def all_done(self):
        return all(self.status[ns][0] in ('parked', 'done', 'failed')
                   for ns in self.robots)

    # --- what RViz shows ------------------------------------------------------
    def _publish_board(self):
        lines = [f'{ns}: {self.status[ns][0]} {self.status[ns][1]}'.rstrip()
                 for ns in self.robots]
        filled = ''.join('X' if i in self._slot_owner else '.'
                         for i in range(len(self.slots)))
        lines.append(f'delivery row [{filled}]  table: '
                     f'{self._delivery_busy or "free"}')
        self.board_pub.publish(String(data='\n'.join(lines)))

    def _marker(self, mid, mtype, pose, scale, rgba, ns='fleet', text=''):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.pose.position.x = float(pose[0])
        m.pose.position.y = float(pose[1])
        m.pose.position.z = float(pose[2])
        yaw = pose[3] if len(pose) > 3 else 0.0
        m.pose.orientation.z = math.sin(yaw / 2.0)
        m.pose.orientation.w = math.cos(yaw / 2.0)
        m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
        m.color.r, m.color.g, m.color.b, m.color.a = (float(v) for v in rgba)

        m.text = text
        return m

    def _publish_markers(self):
        """Draw the job: what is where, who owns what, and what is done."""
        arr = MarkerArray()
        mid = 0

        # collection tables, coloured by the rack they carry
        for name, colour, table, rack, stand in self.collections:
            rgba = COLOUR_RGBA[colour]
            arr.markers.append(self._marker(
                mid, Marker.CUBE, (table[0], table[1], 0.16, table[2]),
                (RT.TABLE_LONG, RT.TABLE_SHORT, 0.32), (*rgba[:3], 0.25)))
            mid += 1
            arr.markers.append(self._marker(
                mid, Marker.TEXT_VIEW_FACING, (table[0], table[1], 1.0),
                (0.0, 0.0, 0.45), (1, 1, 1, 1),
                text=f'{name}\n{colour}'))
            mid += 1
            # the standoff the robot is sent to
            arr.markers.append(self._marker(
                mid, Marker.ARROW, (stand[0], stand[1], 0.05, stand[2]),
                (0.8, 0.12, 0.12), (*rgba[:3], 0.9)))
            mid += 1

        # delivery table and its row of slots
        dx, dy, dyaw = RT.delivery_table_pose()
        arr.markers.append(self._marker(
            mid, Marker.CUBE, (dx, dy, 0.16, dyaw),
            (RT.TABLE_LONG, RT.TABLE_SHORT, 0.32), (0.85, 0.85, 0.9, 0.30)))
        mid += 1
        arr.markers.append(self._marker(
            mid, Marker.TEXT_VIEW_FACING, (dx, dy, 1.0), (0, 0, 0.45),
            (1, 1, 1, 1), text='delivery'))
        mid += 1
        for i, slot, stand in self.slots:
            owner = self._slot_owner.get(i)
            rgba = (0.2, 0.9, 0.2, 0.9) if owner else (0.6, 0.6, 0.6, 0.5)
            arr.markers.append(self._marker(
                mid, Marker.CYLINDER, (slot[0], slot[1], RT.TABLE_TOP + 0.02),
                (0.14, 0.14, 0.04), rgba))
            mid += 1
            arr.markers.append(self._marker(
                mid, Marker.TEXT_VIEW_FACING, (slot[0], slot[1], RT.TABLE_TOP + 0.30),
                (0, 0, 0.16), (1, 1, 1, 1),
                text=f'{i}:{owner}' if owner else f'{i}:free'))
            mid += 1

        # parking triangle
        for i, (px, py, pyaw) in enumerate(self.parks):
            owner = self._park_owner.get(i)
            rgba = (0.95, 0.75, 0.1, 0.9) if owner else (0.5, 0.5, 0.5, 0.4)
            arr.markers.append(self._marker(
                mid, Marker.ARROW, (px, py, 0.05, pyaw), (0.7, 0.12, 0.12), rgba))
            mid += 1
            arr.markers.append(self._marker(
                mid, Marker.TEXT_VIEW_FACING, (px, py, 0.6), (0, 0, 0.30),
                (1, 1, 1, 1), text=f'P{i}:{owner}' if owner else f'P{i}'))
            mid += 1

        board = '\n'.join(f'{ns}: {self.status[ns][0]} {self.status[ns][1]}'.rstrip()
                          for ns in self.robots)
        arr.markers.append(self._marker(
            mid, Marker.TEXT_VIEW_FACING, (0.0, 8.75, 2.6), (0, 0, 0.40),
            (1, 1, 0.6, 1), text=board))
        mid += 1
        self.marker_pub.publish(arr)


def main():
    rclpy.init()
    node = TaskManager()
    ex = rclpy.executors.MultiThreadedExecutor(4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    try:
        node.dispatch()
        while rclpy.ok() and not node.all_done():
            time.sleep(2.0)
        undelivered = [ns for ns in node.robots if ns not in node.delivered]
        node.get_logger().info(
            f'[manager] every robot has finished: {len(node.delivered)}/'
            f'{len(node.robots)} racks delivered')
        for ns in node.robots:
            state, detail = node.status[ns]
            note = '' if ns in node.delivered else '   (rack NOT delivered)'
            node.get_logger().info(f'    {ns}: {state} {detail}{note}')
        if undelivered:
            node.get_logger().error(
                f'[manager] MISSION INCOMPLETE -- no rack from '
                f'{", ".join(undelivered)}')
        # Stay alive so the markers and the status board keep being published.
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
