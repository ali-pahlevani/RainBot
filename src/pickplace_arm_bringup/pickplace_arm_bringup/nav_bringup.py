"""Bring one robot's Nav2 stack up, retrying when bring-up stalls."""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from lifecycle_msgs.srv import GetState
from std_msgs.msg import Bool
from nav2_msgs.srv import ManageLifecycleNodes


ACTIVE = 3          # lifecycle_msgs/State PRIMARY_STATE_ACTIVE


class NavBringup(Node):
    def __init__(self, args):
        super().__init__('nav_bringup')
        self.args = args
        self.ns = args.namespace.strip('/')
        self.manage = self.create_client(
            ManageLifecycleNodes,
            f'/{self.ns}/lifecycle_manager_navigation/manage_nodes')
        self.state_clients = {
            n: self.create_client(GetState, f'/{self.ns}/{n}/get_state')
            for n in args.node
        }

    def _call(self, client, request, timeout):
        fut = client.call_async(request)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def command(self, value, label):
        req = ManageLifecycleNodes.Request()
        req.command = value
        res = self._call(self.manage, req, self.args.command_timeout)
        ok = bool(res and res.success)
        self.get_logger().info(f'[{self.ns}] {label} -> '
                               f'{"ok" if ok else "reported failure"}')
        return ok

    def states(self):
        """Current lifecycle state id of every managed node, None if unknown."""
        out = {}
        for name, cli in self.state_clients.items():
            res = self._call(cli, GetState.Request(), 5.0)
            out[name] = res.current_state.id if res else None
        return out

    def all_active(self, deadline):
        """Poll until every managed node is ACTIVE, or the deadline passes."""
        while time.time() < deadline:
            st = self.states()
            if all(v == ACTIVE for v in st.values()):
                return True, st
            time.sleep(2.0)
        return False, self.states()

    def run(self):
        log = self.get_logger()
        if not self.manage.wait_for_service(timeout_sec=self.args.wait_manager):
            log.error(f'[{self.ns}] manage_nodes never appeared - '
                      f'is the lifecycle manager running?')
            return False

        if self.args.pre_settle > 0.0:
            log.info(f'[{self.ns}] letting the managed nodes construct for '
                     f'{self.args.pre_settle:.0f}s before the first STARTUP')
            time.sleep(self.args.pre_settle)

        for attempt in range(1, self.args.attempts + 1):
            log.info(f'[{self.ns}] bring-up attempt {attempt}'
                     f'/{self.args.attempts}')
            self.command(ManageLifecycleNodes.Request.STARTUP, 'STARTUP')
            ok, st = self.all_active(time.time() + self.args.settle)
            if ok:
                log.info(f'[{self.ns}] all {len(st)} nodes active after '
                         f'{attempt} attempt(s)')
                self.announce_ready()
                return True

            stalled = [n for n, v in st.items() if v != ACTIVE]
            log.warn(f'[{self.ns}] attempt {attempt} left {len(stalled)} node(s) '
                     f'not active: {", ".join(sorted(stalled))}')
            if attempt < self.args.attempts:
                self.command(ManageLifecycleNodes.Request.RESET, 'RESET')
                time.sleep(self.args.backoff)

        log.error(f'[{self.ns}] gave up after {self.args.attempts} attempts; '
                  f'final states: {self.states()}')
        return False


    def announce_ready(self):
        """Announce on a topic that this robot's stack is active."""
        pub = self.create_publisher(Bool, 'nav_ready', 10)
        msg = Bool(data=True)
        self.create_timer(0.5, lambda: pub.publish(msg))
        self.get_logger().info(
            f'[{self.ns}] publishing nav_ready -- the arms, the mission nodes '
            f'and rviz wait on this')


def main(argv=None):
    argv = remove_ros_args(sys.argv if argv is None else argv)[1:]
    p = argparse.ArgumentParser()
    p.add_argument('--namespace', required=True)
    p.add_argument('--node', action='append', default=[],
                   help='managed node name, repeatable')
    p.add_argument('--attempts', type=int, default=4)
    p.add_argument('--settle', type=float, default=45.0,
                   help='seconds to wait for all nodes to reach active')
    p.add_argument('--backoff', type=float, default=5.0)
    p.add_argument('--command-timeout', type=float, default=45.0)
    p.add_argument('--pre-settle', type=float, default=8.0)
    p.add_argument('--wait-manager', type=float, default=120.0)
    args, _unknown = p.parse_known_args(argv)

    rclpy.init()
    node = NavBringup(args)
    try:
        if node.run():
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
