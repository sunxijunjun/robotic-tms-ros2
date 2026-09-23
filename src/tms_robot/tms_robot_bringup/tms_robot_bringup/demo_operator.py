#!/usr/bin/env python3
"""Demo operator: replays the human operator's click sequence via the
execute_task action:

  move_arm_ready                           (staging pose above work area)
  move_arm_above_head                      (camera search pose)
  move_to_tcp_target_offset @ 70 mm        (stepped descent...)
  move_to_tcp_target_offset @ 60 mm
  move_to_tcp_target_offset @ 30 mm
  zero FTS                                 (robotiq_ft_sensor_acc service)
  contact_treatment_test, 30 s hold        (approach + hold + auto-retract)
  move_arm_ready                           (back to staging)
  ... loop forever.

Behavior trees are untouched -- this only presses the same buttons the UI
would. Run with the UI/stack already up and the workspace sourced:
    ros2 run tms_robot_bringup demo_operator
or select "demo_operator" in the UI task dropdown and press Start Task.
The UI Cancel Task button (or canceling the running goal by any other means)
stops the loop: a canceled task raises OperatorCanceled instead of retrying.
"""
import logging
import os
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from robotiq_ft_sensor_interfaces.srv import SensorAccessor
from tms_robot_interfaces.action import ExecuteTask


class OperatorCanceled(Exception):
    """A human canceled the running task (UI cancel button); they win over the demo."""

# Values matched to the current operator session settings.
DESCENT_OFFSETS_M = [0.070, 0.060, 0.030]   # stepped approach offsets
TREATMENT_DURATION_SEC = 30.0
# ponytail: 63 mm (the UI session default) sits ABOVE the contact distance --
# the guard trips at ~63 mm with zero force, so the coil can never reach the
# force band (verified: 3/3 approach failures at exactly the threshold).
# 51 mm is the recorded value from the successful manual loop: contact happens
# around 60 mm, leaving the guard as a ~10 mm backstop below it.
MIN_DISTANCE_M = 0.051
FORCE_MODE = 'LIGHT'
PAUSE_BETWEEN_TASKS_SEC = 2.0
PAUSE_BETWEEN_CYCLES_SEC = 5.0
SEARCH_MAX_ATTEMPTS = 5   # first descent step waits on the camera pose
TREATMENT_MAX_ATTEMPTS = 3  # each retry re-descends, so the approach budget resets
# A move that fails while this BT node is active means the camera never sent a
# target: ask the camera script to save that frame for retraining.
WAIT_TARGET_NODE = 'wait_tcp_target_pose'
MISSED_FRAME_TOPIC = '/camera/save_missed_frame'


LOG_DIR = os.path.expanduser('~/demo_operator_logs')


def setup_file_log():
    os.makedirs(LOG_DIR, exist_ok=True)
    path = os.path.join(LOG_DIR, time.strftime('demo_%Y%m%d_%H%M%S.log'))
    logging.basicConfig(filename=path, level=logging.INFO,
                        format='%(asctime)s %(levelname)s %(message)s')
    return path


class DemoOperator(Node):
    def __init__(self):
        super().__init__('demo_operator')
        self.client = ActionClient(self, ExecuteTask, 'execute_task')
        self.zero_client = self.create_client(SensorAccessor, 'robotiq_ft_sensor_acc')
        self.missed_pub = self.create_publisher(String, MISSED_FRAME_TOPIC, 10)
        self._last_node = None
        self.cycle = 0

    # console + file in one call
    def info(self, msg):
        self.get_logger().info(msg)
        logging.info(msg)

    def warning(self, msg):
        self.get_logger().warning(msg)
        logging.warning(msg)

    def error(self, msg):
        self.get_logger().error(msg)
        logging.error(msg)

    def run_task(self, name, tcp_offset_z_m):
        goal = ExecuteTask.Goal()
        goal.task_name = name
        goal.tcp_offset_z_m = tcp_offset_z_m
        goal.treatment_duration_sec = TREATMENT_DURATION_SEC
        goal.enable_distance_guard = True
        goal.min_distance_m = MIN_DISTANCE_M
        goal.force_mode = FORCE_MODE
        self._last_node = None
        self.info(f'Starting task: {name} (offset {tcp_offset_z_m * 1000:.0f} mm)')
        t0 = time.monotonic()
        send = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.error(f'Task rejected: {name}')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()
        if wrapped.status == GoalStatus.STATUS_CANCELED:
            self.warning(f'Task {name} canceled by operator -- stopping demo')
            raise OperatorCanceled()
        result = wrapped.result
        self.info(f'Task {name}: success={result.success} '
                  f'({result.message}) [{time.monotonic() - t0:.1f}s]')
        if not result.success and self._last_node == WAIT_TARGET_NODE:
            tag = f'cycle{self.cycle}_offset{tcp_offset_z_m * 1000:.0f}mm'
            self.missed_pub.publish(String(data=tag))
            self.info(f'Camera sent no target: requested missed-frame capture ({tag})')
        return result.success

    def on_feedback(self, fb):
        node = fb.feedback.active_node
        if node and node != self._last_node:
            self._last_node = node
            self.info(f'  active: {node}')

    def zero_fts(self):
        if not self.zero_client.wait_for_service(timeout_sec=5.0):
            self.error('FTS zeroing service not available')
            return False
        request = SensorAccessor.Request()
        request.command_id = SensorAccessor.Request.COMMAND_SET_ZERO
        self.info('Zeroing FTS...')
        future = self.zero_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None or not response.success:
            self.error('FTS zeroing failed')
            return False
        self.info(f'FTS zeroed: {response.res}')
        return True

    def run_with_retries(self, name, offset_m, max_attempts):
        for attempt in range(1, max_attempts + 1):
            if self.run_task(name, offset_m):
                time.sleep(PAUSE_BETWEEN_TASKS_SEC)
                return True
            self.warning(f'{name} failed (attempt {attempt}/{max_attempts})')
            time.sleep(PAUSE_BETWEEN_TASKS_SEC)
        return False

    def run_cycle(self):
        # 1. staging pose
        if not self.run_with_retries('move_arm_ready', DESCENT_OFFSETS_M[0], 3):
            return False
        # 2. camera search pose
        if not self.run_with_retries('move_arm_above_head', DESCENT_OFFSETS_M[0], 3):
            return False
        # 3. stepped descent toward the target
        for i, offset_m in enumerate(DESCENT_OFFSETS_M):
            attempts = SEARCH_MAX_ATTEMPTS if i == 0 else 2
            if not self.run_with_retries('move_to_tcp_target_offset', offset_m, attempts):
                return False
        # 4+5. zero FTS at standoff, then approach + 30 s hold + auto retract.
        # ponytail: MAX_TOTAL_ADVANCE workaround lives here, not in the trees --
        # on failure the executor retracts, so we re-descend + re-zero and the
        # approach gets a fresh 50 mm budget each attempt.
        treated = False
        for attempt in range(1, TREATMENT_MAX_ATTEMPTS + 1):
            if not self.zero_fts():
                return False
            time.sleep(1.0)
            if self.run_task('contact_treatment_test', DESCENT_OFFSETS_M[-1]):
                treated = True
                break
            self.warning(
                f'treatment failed (attempt {attempt}/{TREATMENT_MAX_ATTEMPTS}), re-descending')
            if not self.run_with_retries('move_to_tcp_target_offset', DESCENT_OFFSETS_M[-1], 2):
                return False
        if not treated:
            return False
        # 6. back to staging
        time.sleep(PAUSE_BETWEEN_TASKS_SEC)
        return self.run_with_retries('move_arm_ready', DESCENT_OFFSETS_M[0], 3)


def main():
    log_path = setup_file_log()
    rclpy.init()
    demo = DemoOperator()
    demo.info(f'Logging to {log_path}')
    try:
        while True:
            if not demo.client.wait_for_server(timeout_sec=10.0):
                demo.error('execute_task action server not available, retrying...')
                continue
            demo.cycle += 1
            demo.info(f'=== Demo cycle {demo.cycle} starting ===')
            ok = demo.run_cycle()
            # ponytail: no scripted error recovery -- the executor already
            # retracts and restores collision state on failure/cancel, and
            # the next cycle starts from move_arm_ready anyway.
            demo.info(f'=== Demo cycle {demo.cycle} {"complete" if ok else "failed"} ===')
            time.sleep(PAUSE_BETWEEN_CYCLES_SEC)
    except (KeyboardInterrupt, ExternalShutdownException):
        # SIGINT from terminal or the UI Cancel button; rclpy may already be down
        logging.info('Demo stopped by user')
    except OperatorCanceled:
        demo.info('Demo stopped: operator canceled the running task')
    rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
