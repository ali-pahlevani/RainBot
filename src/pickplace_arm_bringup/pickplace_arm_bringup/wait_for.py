"""Block until the sim stack is ready, then exit 0."""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from rclpy.utilities import remove_ros_args
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from rosgraph_msgs.msg import Clock

import tf2_ros


CLOCK_QOS = QoSProfile(depth=10,
                       reliability=ReliabilityPolicy.BEST_EFFORT,
                       durability=DurabilityPolicy.VOLATILE,
                       history=HistoryPolicy.KEEP_LAST)


class WaitFor(Node):
    def __init__(self, args):
        super().__init__('wait_for')
        self.args = args
        self.t0 = time.time()
        self.done = False

        self.clock_seen = False
        self.last_clock = None
        self.stable_since = None
        self.jumps = 0
        self.worst_jump = 0.0

        if args.clock_stable > 0.0:
            self.create_subscription(Clock, '/clock', self._clock_cb, CLOCK_QOS)

        if args.tf:
            self.buf = tf2_ros.Buffer()
            self.listener = tf2_ros.TransformListener(self.buf, self)
        self.tf_ok = not args.tf
        self._topic_seen = set()
        self._topic_subs = {}
        self._tf_seen = set()

        self.create_timer(0.25, self._tick)

    # --- individual checks ---------------------------------------------------
    def _clock_cb(self, msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        now = time.time()
        if not self.clock_seen:
            self.clock_seen = True
            self.stable_since = now
            self.get_logger().info(f'/clock is publishing (t+{now - self.t0:.1f}s)')
        elif t < self.last_clock - self.args.jump_threshold:
            self.jumps += 1
            self.stable_since = now
            self.worst_jump = max(self.worst_jump, self.last_clock - t)
            if self.jumps in (1, 10, 50, 200):
                self.get_logger().warn(
                    f'/clock jumped BACKWARDS by {self.last_clock - t:.2f}s '
                    f'({self.jumps} so far). If this persists, a previous run\'s '
                    f'gz server is probably still running: pkill -9 -f "gz sim"')
        # First message has no previous value to compare against.
        self.last_clock = t if self.last_clock is None else max(self.last_clock, t)

    def _clock_ready(self):
        if self.args.clock_stable <= 0.0:
            return True
        if not self.clock_seen or self.stable_since is None:
            return False
        return (time.time() - self.stable_since) >= self.args.clock_stable

    def _tf_ready(self):
        """True when every requested transform is available."""
        if self.tf_ok:
            return True
        for pair in self.args.tf:
            key = tuple(pair)
            if key in self._tf_seen:
                continue
            try:
                self.buf.lookup_transform(pair[0], pair[1], rclpy.time.Time())
            except Exception:
                return False
            self._tf_seen.add(key)
            self.get_logger().info(
                f'TF {pair[0]}->{pair[1]} available '
                f'(t+{time.time() - self.t0:.1f}s)')
        self.tf_ok = True
        return True

    def _topics_ready(self):
        """A topic counts as ready once a message has actually arrived on it."""
        ready = True
        for t in self.args.topic:
            if t in self._topic_seen:
                continue
            ready = False
            if t not in self._topic_subs:
                types = dict(self.get_topic_names_and_types()).get(t)
                if not types:
                    continue                 # not advertised yet
                try:
                    msg_cls = get_message(types[0])
                except Exception:            # type not resolvable yet
                    continue
                qos = QoSProfile(depth=1,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
                self._topic_subs[t] = self.create_subscription(
                    msg_cls, t, lambda _m, k=t: self._topic_seen.add(k), qos)
        return ready

    def _services_ready(self):
        names = {n for n, _ in self.get_service_names_and_types()}
        return all(s in names for s in self.args.service)

    def _actions_ready(self):
        names = {n for n, _ in self.get_service_names_and_types()}
        return all(f'{a}/_action/send_goal' in names for a in self.args.action)

    def _nodes_ready(self):
        live = set(self.get_node_names())
        return all(n.lstrip('/') in live for n in self.args.node)

    # --- driver --------------------------------------------------------------
    def _tick(self):
        if self.done:
            return
        elapsed = time.time() - self.t0

        checks = (('clock', self._clock_ready()), ('tf', self._tf_ready()),
                  ('topics', self._topics_ready()),
                  ('services', self._services_ready()),
                  ('actions', self._actions_ready()),
                  ('nodes', self._nodes_ready()))

        if all(ok for _, ok in checks):
            self.done = True
            extra = (f' ({self.jumps} real clock jump-backs, worst '
                     f'{self.worst_jump:.2f}s)' if self.jumps else '')
            self.get_logger().info(
                f'[{self.args.label}] ready after {elapsed:.1f}s{extra}')
            raise SystemExit(0)

        if elapsed >= self.args.timeout:
            self.done = True
            pending = ', '.join(n for n, ok in checks if not ok)
            self.get_logger().warn(
                f'[{self.args.label}] TIMEOUT after {elapsed:.1f}s waiting on: '
                f'{pending}. Continuing anyway.')
            raise SystemExit(0)

        # Progress note roughly every 5 s so a long wait is never silent.
        if int(elapsed * 4) % 20 == 0 and elapsed >= 5.0:
            pending = ', '.join(n for n, ok in checks if not ok)
            self.get_logger().info(
                f'[{self.args.label}] waiting {elapsed:.0f}s on: {pending}')


def main(argv=None):
    argv = remove_ros_args(args=sys.argv)[1:] if argv is None else argv
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--label', default='wait_for')
    p.add_argument('--clock-stable', type=float, default=0.0,
                   help='require /clock free of REAL backward jumps for this '
                        'many seconds before declaring ready')
    p.add_argument('--jump-threshold', type=float, default=0.1,
                   help='backward step (s) that counts as a real jump. /clock '
                        'is BEST_EFFORT over UDP, so consecutive messages get '
                        'delivered out of order routinely: measured live, every '
                        'backward step was exactly one 10 ms sim tick (max 20 '
                        'ms) while the clock advanced 113 s net over the same '
                        'window. Counting those as resets made this gate block '
                        'forever. A genuine fault -- an orphaned second gz '
                        'server, a sim reset -- moves time by whole seconds, so '
                        '0.1 s separates the two cleanly with 5x margin.')
    p.add_argument('--tf', nargs=2, action='append', default=[],
                   metavar=('TARGET', 'SOURCE'))
    p.add_argument('--topic', action='append', default=[])
    p.add_argument('--service', action='append', default=[])
    p.add_argument('--action', action='append', default=[])
    p.add_argument('--node', action='append', default=[])
    p.add_argument('--timeout', type=float, default=120.0)
    args, _unknown = p.parse_known_args(argv)

    rclpy.init()
    node = WaitFor(args)
    code = 0
    try:
        rclpy.spin(node)
    except SystemExit as exc:
        code = exc.code or 0
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
