#!/usr/bin/env python3
"""Capture new 'ready': current TCP pose lifted +0.20 m world-vertical.
FK current joints -> add z -> IK seeded from current joints -> print joints.
No robot motion.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK, GetPositionIK

LIFT = 0.40
GROUP = 'ur_arm'
TCP = 'ur10e_tcp'


class Cap(Node):
    def __init__(self):
        super().__init__('capture_ready')
        self.js = None
        self.create_subscription(JointState, '/joint_states', self._cb, 1)
        self.fk = self.create_client(GetPositionFK, '/compute_fk')
        self.ik = self.create_client(GetPositionIK, '/compute_ik')

    def _cb(self, m):
        self.js = m

    def call(self, cli, req):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        return fut.result()


def main():
    rclpy.init()
    n = Cap()
    assert n.fk.wait_for_service(timeout_sec=5.0) and n.ik.wait_for_service(timeout_sec=5.0)
    while n.js is None:
        rclpy.spin_once(n, timeout_sec=1.0)

    fkr = GetPositionFK.Request()
    fkr.fk_link_names = [TCP]
    fkr.robot_state.joint_state.name = list(n.js.name)
    fkr.robot_state.joint_state.position = list(n.js.position)
    fres = n.call(n.fk, fkr)
    assert fres and fres.error_code.val == 1, f"FK failed: {fres and fres.error_code.val}"
    pose = fres.pose_stamped[0]
    print(f"current TCP in '{pose.header.frame_id}': "
          f"[{pose.pose.position.x:.3f}, {pose.pose.position.y:.3f}, {pose.pose.position.z:.3f}]")
    pose.pose.position.z += LIFT

    ikr = GetPositionIK.Request()
    ikr.ik_request.group_name = GROUP
    ikr.ik_request.ik_link_name = TCP
    ikr.ik_request.pose_stamped = pose
    ikr.ik_request.robot_state.joint_state.name = list(n.js.name)
    ikr.ik_request.robot_state.joint_state.position = list(n.js.position)
    ikr.ik_request.avoid_collisions = True
    ikr.ik_request.timeout.sec = 2
    ires = n.call(n.ik, ikr)
    assert ires and ires.error_code.val == 1, f"IK failed: {ires and ires.error_code.val}"
    sol = dict(zip(ires.solution.joint_state.name, ires.solution.joint_state.position))
    cur = dict(zip(n.js.name, n.js.position))
    print("\njoint            current   ->  lifted   (delta deg)")
    for j in sorted(cur):
        d = (sol[j] - cur[j]) * 57.2958
        print(f"{j:32s} {cur[j]:8.4f} -> {sol[j]:8.4f}  ({d:+6.1f})")
        assert abs(sol[j]) < 3.1416, f"{j} solution out of +/-pi bounds"
        assert abs(d) < 60, f"{j} jumped {d:.0f} deg — IK flipped branch, do not use"
    print("\nSRDF values:")
    for j in sorted(sol):
        if j in cur:
            print(f'        <joint name="{j}" value="{sol[j]:.4f}"/>')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
