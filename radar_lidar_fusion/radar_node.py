#!/usr/bin/env python3

import serial

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray


MULTI_TARGET_CMD = bytes([
    0xFD, 0xFC, 0xFB, 0xFA,
    0x02, 0x00,
    0x90, 0x00,
    0x04, 0x03, 0x02, 0x01
])


def decode_coord(raw):
    if raw & 0x8000:
        return raw & 0x7FFF
    return -(raw & 0x7FFF)


def decode_speed(raw):
    if raw & 0x8000:
        return raw & 0x7FFF
    return -(raw & 0x7FFF)


class RadarNode(Node):

    def __init__(self):
        super().__init__('radar_node')

        # Publisher
        self.publisher = self.create_publisher(
            PoseArray,
            '/radar_targets',
            10
        )

        # Open UART
        try:

            self.ser = serial.Serial(
                '/dev/ttyAMA0',
                256000,
                timeout=1
            )

            # Enable multi-target mode
            self.ser.write(MULTI_TARGET_CMD)
            self.ser.flush()

            self.get_logger().info(
                'RD03D connected on /dev/ttyAMA0'
            )

        except Exception as e:

            self.get_logger().error(
                f'Failed to open serial port: {e}'
            )

            raise

        # Radar updates at approximately 20 Hz
        self.timer = self.create_timer(
            0.05,
            self.read_radar
        )

    def read_radar(self):

        frame = self.ser.read_until(b'\x55\xCC')

        if len(frame) < 30:
            return

        if frame[0] != 0xAA or frame[1] != 0xFF:
            return

        pose_array = PoseArray()

        pose_array.header.stamp = (
            self.get_clock().now().to_msg()
        )

        pose_array.header.frame_id = "radar_frame"

        valid_targets = 0

        for target in range(3):

            idx = 4 + target * 8

            x_raw = frame[idx] | (frame[idx + 1] << 8)
            y_raw = frame[idx + 2] | (frame[idx + 3] << 8)

            speed_raw = (
                frame[idx + 4]
                | (frame[idx + 5] << 8)
            )

            reserved = (
                frame[idx + 6]
                | (frame[idx + 7] << 8)
            )

            x = decode_coord(x_raw)
            y = decode_coord(y_raw)
            speed = decode_speed(speed_raw)

            if (
                x == 0
                and y == 0
                and speed == 0
                and reserved == 0
            ):
                continue

            pose = Pose()

            pose.position.x = y / 1000.0
            pose.position.y = x / 1000.0
            pose.position.z = 0.0

            # Orientation is currently unused
            pose.orientation.x = 0.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 1.0

            pose_array.poses.append(pose)

            valid_targets += 1

            self.get_logger().debug(
                f'Target {target + 1}: '
                f'x={pose.position.x:.2f} m, '
                f'y={pose.position.y:.2f} m, '
                f'speed={speed/1000.0:.2f} m/s'
            )

        self.publisher.publish(
            pose_array
        )

        self.get_logger().debug(
            f'Published {valid_targets} target(s)'
        )

    def destroy_node(self):

        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = RadarNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()