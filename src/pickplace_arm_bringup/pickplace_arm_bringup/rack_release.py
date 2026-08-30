#!/usr/bin/env python3
"""Break the startup welds between every robot's gripper and every rack."""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Empty


class RackRelease(Node):
    def __init__(self, robots, models, duration, rate):
        super().__init__('rack_release')
        self.duration = duration
        self.rate = rate
        self.pubs = {}
        for ns in robots:
            for model in models:
                topic = f'/{ns}/{model}/detach'
                self.pubs[topic] = self.create_publisher(Empty, topic, 10)
        self.get_logger().info(
            f'[release] {len(self.pubs)} welds to break '
            f'({len(robots)} robots x {len(models)} racks), '
            f'publishing for {duration:.0f}s at {rate:.0f} Hz')

    def run(self):
        deadline = time.time() + 10.0
        while time.time() < deadline:
            if all(p.get_subscription_count() > 0 for p in self.pubs.values()):
                break
            time.sleep(0.2)
        unmatched = [t for t, p in self.pubs.items()
                     if p.get_subscription_count() == 0]
        if unmatched:
            self.get_logger().warn(
                f'[release] {len(unmatched)} detach topics have no subscriber '
                f'(is the ros_gz bridge up?): {unmatched[:3]}')

        msg = Empty()
        sent = 0
        end = time.time() + self.duration
        while time.time() < end:
            for pub in self.pubs.values():
                pub.publish(msg)
                sent += 1
            time.sleep(1.0 / self.rate)
        self.get_logger().info(
            f'[release] sent {sent} detach messages across {len(self.pubs)} topics')


def main(argv=None):
    argv = remove_ros_args(args=(argv if argv is not None else sys.argv))
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--robot', action='append', default=[],
                    help='robot namespace; repeat for each')
    ap.add_argument('--model', action='append', default=[],
                    help='graspable model name; repeat for each')
    ap.add_argument('--duration', type=float, default=15.0)
    ap.add_argument('--rate', type=float, default=2.0)
    args = ap.parse_args(argv[1:])
    if not args.robot or not args.model:
        print('rack_release: nothing to do (no --robot or no --model)')
        return 0

    rclpy.init()
    node = RackRelease(args.robot, args.model, args.duration, args.rate)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
