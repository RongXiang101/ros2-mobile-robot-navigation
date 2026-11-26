#!/usr/bin/env python3

import copy
import rclpy

from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


class SensorCovarianceRelay(Node):

    def __init__(self):
        super().__init__('sensor_covariance_relay')

        self.odom_pub = self.create_publisher(
            Odometry, '/wheel/odom', 10
        )

        self.imu_pub = self.create_publisher(
            Imu, '/imu/data_cov', 50
        )

        self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 50
        )

        self.get_logger().info(
            'Covariance relay started.'
        )

    def odom_callback(self, msg):

        out = copy.deepcopy(msg)

        out.twist.covariance = [0.0] * 36

        # Provisional effective measurement variances
        out.twist.covariance[0] = 3.0e-5
        out.twist.covariance[7] = 1.0e-5
        out.twist.covariance[35] = 2.1e-5

        self.odom_pub.publish(out)

    def imu_callback(self, msg):

        out = copy.deepcopy(msg)

        # Gyroscope covariance
        out.angular_velocity_covariance = [0.0] * 9

        out.angular_velocity_covariance[0] = 1.0223e-4
        out.angular_velocity_covariance[4] = 1.0038e-4
        out.angular_velocity_covariance[8] = 9.8510e-5

        # Accelerometer covariance
        out.linear_acceleration_covariance = [0.0] * 9

        out.linear_acceleration_covariance[0] = 0.00244867
        out.linear_acceleration_covariance[4] = 0.00248871
        out.linear_acceleration_covariance[8] = 0.00252838

        self.imu_pub.publish(out)


def main(args=None):

    rclpy.init(args=args)

    node = SensorCovarianceRelay()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
