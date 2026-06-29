#!/usr/bin/env python3

import serial

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range


class LidarNode(Node):

    def __init__(self):
        super().__init__('lidar_node')

        # Publisher
        self.publisher = self.create_publisher(
            Range,
            '/lidar_range',
            10
        )

        # Open serial port
        try:

            self.ser = serial.Serial(
                '/dev/ttyAMA1',
                115200,
                timeout=1
            )

            self.get_logger().info(
                'TF-Luna connected on /dev/ttyAMA1'
            )

        except Exception as e:

            self.get_logger().error(
                f'Failed to open serial port: {e}'
            )

            raise

        # TF-Luna updates around 100 Hz
        self.timer = self.create_timer(
            0.01,
            self.read_lidar
        )

    def read_lidar(self):

        # Look for TF-Luna frame header
        if self.ser.read() != b'\x59':
            return

        if self.ser.read() != b'\x59':
            return

        data = self.ser.read(7)

        if len(data) != 7:
            return

        distance_cm = data[0] + (data[1] << 8)
        strength = data[2] + (data[3] << 8)

        distance_m = distance_cm / 100.0

        # Ignore invalid readings
        if distance_m < 0.20 or distance_m > 8.00:
            self.get_logger().warning(
                f'Ignoring invalid range: {distance_m:.2f} m'
            )
            return

        msg = Range()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'lidar_frame'

        msg.radiation_type = Range.INFRARED

        msg.field_of_view = 0.04

        msg.min_range = 0.20
        msg.max_range = 8.00

        msg.range = distance_m

        self.publisher.publish(msg)

        self.get_logger().debug(
            f'Distance: {distance_m:.2f} m | '
            f'Strength: {strength}'
        )

    def destroy_node(self):

        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = LidarNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()