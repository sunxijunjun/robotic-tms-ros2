#!/usr/bin/env python3
"""Compute the same -0.10m TCP-Z Cartesian retract the tree runs and check the
resulting joint path for configuration jumps (no motion, service call only)."""
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetCartesianPath, GetPositionFK


class Chk(Node):
    def __init__(self):
        super().__init__('retract_check')
        self.js = None
        self.create_subscription(JointState, '/joint_states', lambda m: setattr(self, 'js', m), 1)
        self.cart = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.fk = self.create_client(GetPositionFK, '/compute_fk')

    def call(self, cli, req):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        return fut.result()


def main():
    rclpy.init()
    n = Chk()
    assert n.cart.wait_for_service(timeout_sec=5.0) and n.fk.wait_for_service(timeout_sec=5.0)
    while n.js is None:
        rclpy.spin_once(n, timeout_sec=1.0)

    fkr = GetPositionFK.Request()
    fkr.fk_link_names = ['ur10e_tcp']
    fkr.robot_state.joint_state.name = list(n.js.name)
    fkr.robot_state.joint_state.position = list(n.js.position)
    f = n.call(n.fk, fkr)
    assert f and f.error_code.val == 1
    pose = f.pose_stamped[0].pose
    q = pose.orientation
    # TCP -Z axis in world = -R.col(2)
    zx = 2 * (q.x * q.z + q.w * q.y)
    zy = 2 * (q.y * q.z - q.w * q.x)
    zz = 1 - 2 * (q.x * q.x + q.y * q.y)
    tgt = GetCartesianPath.Request()
    tgt.header.frame_id = f.pose_stamped[0].header.frame_id
    tgt.group_name = 'ur_arm'
    tgt.link_name = 'ur10e_tcp'
    tgt.start_state.joint_state.name = list(n.js.name)
    tgt.start_state.joint_state.position = list(n.js.position)
    import copy
    wp = copy.deepcopy(pose)
    wp.position.x -= 0.10 * zx
    wp.position.y -= 0.10 * zy
    wp.position.z -= 0.10 * zz
    tgt.waypoints = [wp]
    tgt.max_step = 0.001
    tgt.avoid_collisions = True
    r = n.call(n.cart, tgt)
    assert r, 'cartesian service failed'
    print(f"fraction: {r.fraction*100:.1f}%  error_code: {r.error_code.val}")
    jt = r.solution.joint_trajectory
    names = jt.joint_names
    prev = None
    max_step = {j: 0.0 for j in names}
    for p in jt.points:
        if prev:
            for j, a, b in zip(names, prev, p.positions):
                max_step[j] = max(max_step[j], abs(b - a))
        prev = p.positions
    print(f"{len(jt.points)} points; max single-step joint change (deg):")
    for j in names:
        flag = '  <-- JUMP' if max_step[j] > math.radians(5) else ''
        print(f"  {j:32s} {math.degrees(max_step[j]):8.2f}{flag}")
    if jt.points:
        print("start pan: %.4f  end pan: %.4f" % (jt.points[0].positions[names.index('ur10e_shoulder_pan_joint')],
                                                  jt.points[-1].positions[names.index('ur10e_shoulder_pan_joint')]))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
