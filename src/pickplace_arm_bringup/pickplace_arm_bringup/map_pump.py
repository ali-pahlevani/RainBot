"""Re-publish the shared /map periodically while the fleet is coming up."""
import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from nav_msgs.msg import OccupancyGrid

# Matches map_server's own publisher, and what every subscriber here expects.
MAP_QOS = QoSProfile(depth=1,
                     history=HistoryPolicy.KEEP_LAST,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


class MapPump(Node):
    def __init__(self, period, duration):
        super().__init__('map_pump')
        self._map = None
        self._sent = 0
        self._duration = duration
        self._done = False
        self._burst = 20.0
        self._last_count = 0
        self._start = self.get_clock().now()
        self._pub = self.create_publisher(OccupancyGrid, '/map', MAP_QOS)
        self._sub = self.create_subscription(
            OccupancyGrid, '/map', self._on_map, MAP_QOS)
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f'[map_pump] re-publishing /map every {period:.1f}s for '
            f'{duration:.0f}s once the map arrives')

    def _on_map(self, msg):
        # Our own re-publications come back here too; only the first matters.
        if self._map is None:
            self._map = msg
            self.get_logger().info(
                f'[map_pump] got the map ({msg.info.width}x{msg.info.height}) '
                f'-- pumping it to any subscriber that missed it')

    def _tick(self):
        """Publish only when a new subscriber has appeared."""
        if self._map is None or self._done:
            return
        elapsed = (self.get_clock().now() - self._start).nanoseconds / 1e9
        if elapsed > self._duration:
            self._done = True
            self.get_logger().info(
                f'[map_pump] done after {self._sent} republishes; the latched '
                f'copy stays available')
            return
        count = self._pub.get_subscription_count()
        if elapsed < self._burst or count > self._last_count:
            if count != self._last_count:
                self.get_logger().info(
                    f'[map_pump] /map subscribers {self._last_count} -> {count}'
                    f' -- republishing')
            self._pub.publish(self._map)
            self._sent += 1
        self._last_count = count


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--period', type=float, default=2.0)
    ap.add_argument('--duration', type=float, default=300.0)
    args, _ = ap.parse_known_args()
    rclpy.init()
    node = MapPump(args.period, args.duration)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
