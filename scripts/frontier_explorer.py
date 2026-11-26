#!/usr/bin/env python3
# Modified from upstream to precheck paths and track spatial goal cooldowns and navigation results.

import math
import time

import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from action_msgs.msg import GoalStatus


UNKNOWN = -1
FREE = 0


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )

        self.plan_client = ActionClient(
            self,
            ComputePathToPose,
            'compute_path_to_pose'
        )

        self.candidates = []
        self.candidate_index = 0
        self.max_checks_per_round = 8
        self.checks_this_round = 0
        self.filtered_this_round = 0
        self.rejected_this_round = 0

        self.map = None
        self.map_info = None
        self.exploring = False

        # 当前导航目标
        self.active_goal = None

        # 已尝试目标的临时排除记录
        # 每条记录: (x, y, radius, expire_time)
        self.blocked_goals = []

        # 冷却时间，单位：秒（单调时钟）
        self.cooldown = 180.0

        # 最近一次无候选目标的日志时间
        self.last_no_goal_log = 0.0

        self.get_logger().info(
            'Frontier Explorer started with goal filtering.'
        )

    def map_callback(self, msg):

        if self.exploring:
            return

        self.map = np.array(msg.data).reshape(
            (msg.info.height, msg.info.width)
        )
        self.map_info = msg.info

        frontiers = self.detect_frontiers()

        if not frontiers:
            self.log_no_goal('No frontiers found.')
            return

        # 删除已经过期的排除记录
        now = time.monotonic()

        self.blocked_goals = [
            record
            for record in self.blocked_goals
            if record[3] > now
        ]

        self.candidates = frontiers
        self.candidate_index = 0
        self.checks_this_round = 0
        self.filtered_this_round = 0
        self.rejected_this_round = 0
        self.exploring = True

        self.try_next_candidate()

    def detect_frontiers(self):

        frontiers = []

        h, w = self.map.shape

        for y in range(1, h - 1):
            for x in range(1, w - 1):

                if self.map[y, x] == FREE:

                    neighbors = self.map[
                        y - 1:y + 2,
                        x - 1:x + 2
                    ]

                    if np.any(neighbors == UNKNOWN):
                        frontiers.append((x, y))

        return frontiers

    def frontier_to_goal(self, cell):

        x, y = cell
        origin = self.map_info.origin
        q = origin.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y ** 2 + q.z ** 2),
        )
        local_x = (x + 0.5) * self.map_info.resolution
        local_y = (y + 0.5) * self.map_info.resolution
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = origin.position.x + cos_yaw * local_x - sin_yaw * local_y
        goal.pose.position.y = origin.position.y + sin_yaw * local_x + cos_yaw * local_y
        goal.pose.orientation.w = 1.0

        return goal

    def is_blocked(self, goal):

        x = goal.pose.position.x
        y = goal.pose.position.y

        for bx, by, radius, _ in self.blocked_goals:

            distance = np.hypot(
                x - bx,
                y - by
            )

            if distance < radius:
                return True

        return False

    def remember_goal(self, radius):

        if self.active_goal is None:
            return

        x = self.active_goal.pose.position.x
        y = self.active_goal.pose.position.y

        expire_time = time.monotonic() + self.cooldown

        self.blocked_goals.append(
            (x, y, radius, expire_time)
        )

    def try_next_candidate(self):

        if not self.plan_client.server_is_ready():
            self.get_logger().warn(
                'ComputePathToPose action server is not ready.'
            )
            self.exploring = False
            self.active_goal = None
            return

        while self.candidate_index < len(self.candidates):

            if self.checks_this_round >= self.max_checks_per_round:
                break

            cell = self.candidates[self.candidate_index]
            self.candidate_index += 1

            goal = self.frontier_to_goal(cell)

            if self.is_blocked(goal):
                self.filtered_this_round += 1
                continue

            self.active_goal = goal
            self.checks_this_round += 1

            self.check_path(goal)
            return

        check_limit_reached = self.checks_this_round >= self.max_checks_per_round
        self.get_logger().info(
            'No acceptable path in this frontier search round: '
            f'frontiers={len(self.candidates)}, filtered={self.filtered_this_round}, '
            f'checks={self.checks_this_round}/{self.max_checks_per_round}, '
            f'planning_rejections={self.rejected_this_round}, '
            f'check_limit_reached={check_limit_reached}, '
            f'unexamined={len(self.candidates) - self.candidate_index}.'
        )

        self.exploring = False
        self.active_goal = None

    def check_path(self, goal_pose):

        goal_pose.header.stamp = (
            self.get_clock().now().to_msg()
        )

        x = goal_pose.pose.position.x
        y = goal_pose.pose.position.y

        self.get_logger().info(
            f'Checking path to x={x:.3f}, y={y:.3f}'
        )

        request = ComputePathToPose.Goal()
        request.goal = goal_pose
        request.planner_id = 'GridBased'
        request.use_start = False

        future = self.plan_client.send_goal_async(request)

        future.add_done_callback(
            self.plan_response_callback
        )

    def plan_response_callback(self, future):

        try:
            handle = future.result()

            if not handle.accepted:
                self.reject_candidate('Planning request rejected.')
                return

            result_future = handle.get_result_async()

            result_future.add_done_callback(
                self.plan_result_callback
            )

        except Exception as e:
            self.reject_candidate(f'Planning response error: {e}')

    def plan_result_callback(self, future):

        try:
            response = future.result()

            if response.status != GoalStatus.STATUS_SUCCEEDED:
                self.reject_candidate('Path planning failed.')
                return

            poses = response.result.path.poses

            if len(poses) < 2:
                self.reject_candidate('Path contains fewer than 2 poses.')
                return

            end = poses[-1].pose.position
            goal = self.active_goal.pose.position

            end_error = float(np.hypot(
                end.x - goal.x,
                end.y - goal.y
            ))

            if end_error > 0.1:
                self.reject_candidate(
                    f'Path endpoint error: {end_error:.3f} m.'
                )
                return

            self.get_logger().info(
                f'Path accepted: {len(poses)} poses, '
                f'endpoint error={end_error:.3f} m.'
            )

        except Exception as e:
            self.reject_candidate(f'Planning result error: {e}')
            return

        self.send_goal(self.active_goal)

    def reject_candidate(self, reason):

        self.rejected_this_round += 1
        self.get_logger().warn(reason)

        # 排除该候选附近的区域，避免紧邻栅格重复规划。
        self.remember_goal(radius=0.65)

        self.active_goal = None

        # 保持 exploring=True，防止地图回调打断候选检查。
        self.try_next_candidate()

    def send_goal(self, goal_pose):

        if not self.nav_client.server_is_ready():
            self.get_logger().warn(
                'NavigateToPose action server is not ready. Retrying on a later map update.',
                throttle_duration_sec=10.0,
            )
            self.exploring = False
            self.active_goal = None
            return

        x = goal_pose.pose.position.x
        y = goal_pose.pose.position.y

        self.get_logger().info(
            f'Sending frontier goal: x={x:.3f}, y={y:.3f}'
        )

        goal = NavigateToPose.Goal()
        goal.pose = goal_pose

        try:
            future = self.nav_client.send_goal_async(goal)
        except Exception as e:
            self.get_logger().error(f'Failed to send navigation goal: {e}')
            self.exploring = False
            self.active_goal = None
            return

        future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):

        try:
            goal_handle = future.result()

            if not goal_handle.accepted:

                self.get_logger().warn(
                    'Frontier goal rejected.'
                )

                self.remember_goal(radius=0.6)

                self.exploring = False
                self.active_goal = None
                return

            result_future = goal_handle.get_result_async()

            result_future.add_done_callback(
                self.result_callback
            )

        except Exception as e:

            self.get_logger().error(
                f'Goal response error: {e}'
            )

            self.remember_goal(radius=0.6)

            self.exploring = False
            self.active_goal = None

    def result_callback(self, future):

        try:
            result = future.result()

            status = result.status

            if status == GoalStatus.STATUS_SUCCEEDED:

                self.get_logger().info(
                    'Frontier goal succeeded.'
                )

                # 防止同一位置立即重复成功
                self.remember_goal(radius=0.45)

            elif status == GoalStatus.STATUS_ABORTED:

                self.get_logger().warn(
                    'Frontier goal aborted.'
                )

                # 失败目标附近暂时不再尝试
                self.remember_goal(radius=0.65)

            elif status == GoalStatus.STATUS_CANCELED:

                self.get_logger().warn(
                    'Frontier goal canceled.'
                )

                self.remember_goal(radius=0.6)

            else:

                self.get_logger().warn(
                    f'Frontier goal ended with status {status}.'
                )

                self.remember_goal(radius=0.6)

        except Exception as e:

            self.get_logger().error(
                f'Navigation result error: {e}'
            )

            self.remember_goal(radius=0.6)

        finally:
            self.exploring = False
            self.active_goal = None

    def log_no_goal(self, message):

        now = time.monotonic()

        if now - self.last_no_goal_log >= 10.0:

            self.get_logger().info(message)

            self.last_no_goal_log = now


def main():

    rclpy.init()

    node = FrontierExplorer()

    try:
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
