#!/usr/bin/env python3
"""Plan-only check: where does the TCP travel during move_arm_ready legs?
Calls /plan_kinematic_path (no execution), then /compute_fk per waypoint,
expressed in the dummy_head frame. Prints TCP distance-to-head along the path.
"""
import sys
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetMotionPlan, GetPositionFK
from moveit_msgs.msg import Constraints, JointConstraint, RobotState

UP = {  # SRDF 'up'
    'ur10e_shoulder_pan_joint': 0.0, 'ur10e_shoulder_lift_joint': -1.5708,
    'ur10e_elbow_joint': 0.0, 'ur10e_wrist_1_joint': -1.5708,
    'ur10e_wrist_2_joint': 0.0, 'ur10e_wrist_3_joint': 0.0}
READY = {  # SRDF 'ready' (40cm lifted, commit 44f349c)
    'ur10e_shoulder_pan_joint': 2.2924, 'ur10e_shoulder_lift_joint': -2.3482,
    'ur10e_elbow_joint': 1.8037, 'ur10e_wrist_1_joint': -1.6932,
    'ur10e_wrist_2_joint': -0.9580, 'ur10e_wrist_3_joint': -0.6308}


class Check(Node):
    def __init__(self):
        super().__init__('plan_check')
        self.js = None
        self.sub = self.create_subscription(JointState, '/joint_states', self._cb, 1)
        self.plan_cli = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.fk_cli = self.create_client(GetPositionFK, '/compute_fk')

    def _cb(self, msg):
        self.js = msg

    def call(self, cli, req):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        return fut.result()

    def plan(self, start_state, goal):
        req = GetMotionPlan.Request()
        mr = req.motion_plan_request
        mr.group_name = 'ur_arm'
        mr.pipeline_id = 'pilz_industrial_motion_planner'
        mr.planner_id = 'PTP'
        mr.allowed_planning_time = 5.0
        mr.max_velocity_scaling_factor = 0.1
        mr.max_acceleration_scaling_factor = 0.1
        mr.start_state = start_state
        c = Constraints()
        for name, pos in goal.items():
            jc = JointConstraint(joint_name=name, position=pos,
                                 tolerance_above=1e-4, tolerance_below=1e-4, weight=1.0)
            c.joint_constraints.append(jc)
        mr.goal_constraints = [c]
        res = self.call(self.plan_cli, req)
        assert res and res.motion_plan_response.error_code.val == 1, \
            f"planning failed: {res and res.motion_plan_response.error_code.val}"
        return res.motion_plan_response.trajectory.joint_trajectory

    def fk_head_frame(self, names, positions):
        req = GetPositionFK.Request()
        req.header.frame_id = 'dummy_head'
        req.fk_link_names = ['ur10e_tcp']
        req.robot_state.joint_state.name = list(names)
        req.robot_state.joint_state.position = list(positions)
        res = self.call(self.fk_cli, req)
        assert res and res.error_code.val == 1, f"FK failed: {res and res.error_code.val}"
        p = res.pose_stamped[0].pose.position
        return p.x, p.y, p.z


def main():
    rclpy.init()
    n = Check()
    for cli in (n.plan_cli, n.fk_cli):
        assert cli.wait_for_service(timeout_sec=5.0), 'moveit services unavailable'
    while n.js is None:
        rclpy.spin_once(n, timeout_sec=1.0)

    legs = [('current -> ready', dict(zip(n.js.name, n.js.position)), READY)]
    for label, s, g in legs:
        ss = RobotState()
        ss.joint_state.name = list(s.keys())
        ss.joint_state.position = list(s.values())
        traj = n.plan(ss, g)
        pts = traj.points
        step = max(1, len(pts) // 25)
        print(f"\n=== {label}: {len(pts)} waypoints, "
              f"{pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec/1e9:.1f}s ===")
        print(f"{'t[s]':>6} {'x':>7} {'y':>7} {'z':>7} {'dist_to_head[m]':>15}")
        mind, mint = 1e9, 0.0
        for p in pts[::step] + [pts[-1]]:
            x, y, z = n.fk_head_frame(traj.joint_names, p.positions)
            d = (x * x + y * y + z * z) ** 0.5
            t = p.time_from_start.sec + p.time_from_start.nanosec / 1e9
            if d < mind:
                mind, mint = d, t
            print(f"{t:6.1f} {x:7.3f} {y:7.3f} {z:7.3f} {d:15.3f}")
        print(f"--- min distance to head: {mind:.3f} m at t={mint:.1f}s")
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
