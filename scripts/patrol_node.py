#!/usr/bin/env python3
# Modified from upstream to run one waypoint pass and stop on failure or cancellation.
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from math import sin, cos
import time

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        self.navigator = BasicNavigator()
        
        self.waypoints = [
            self.create_pose(1.0, 0.0, 0.0),
            self.create_pose(2.0, 1.0, 1.57),
            self.create_pose(0.0, 1.0, 3.14),
        ]
        self.current_waypoint = 0
        
    def create_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = sin(yaw / 2)
        pose.pose.orientation.w = cos(yaw / 2)
        return pose
        
    def run_patrol(self):
        from nav2_simple_commander.robot_navigator import TaskResult

        self.navigator.waitUntilNav2Active()

         # 第一版：完成一轮巡航后退出
        for i, goal in enumerate(self.waypoints):

            if not rclpy.ok():
                break

            goal.header.stamp = self.navigator.get_clock().now().to_msg()

            self.get_logger().info(f'Going to waypoint {i}')
            self.navigator.goToPose(goal)

            while rclpy.ok() and not self.navigator.isTaskComplete():
                time.sleep(0.1)

            if not rclpy.ok():
                self.navigator.cancelTask()
                break

            result = self.navigator.getResult()

            if result == TaskResult.SUCCEEDED:
                self.get_logger().info(f'Waypoint {i} succeeded.')

            elif result == TaskResult.CANCELED:
                self.get_logger().warn(f'Waypoint {i} canceled.')
                break

            else:
                self.get_logger().error(f'Waypoint {i} failed.')
                break

        self.get_logger().info('Patrol finished.')

def main():
    rclpy.init()
    node = PatrolNode()
    node.run_patrol()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
