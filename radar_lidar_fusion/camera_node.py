import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        # Publisher
        self.publisher = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )

        # CV Bridge
        self.bridge = CvBridge()

        # Parameters (override with --ros-args -p device_index:=0)
        device_index = self.declare_parameter('device_index', 8).value

        # Open camera
        self.cap = cv2.VideoCapture(device_index, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().error(
                f'Failed to open /dev/video{device_index}'
            )
            return

        # Camera settings
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_FPS, 30)

        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.cap.get(cv2.CAP_PROP_FPS)

        self.get_logger().info(
            f'Camera opened: {width}x{height} @ {fps:.1f} FPS'
        )

        # Publish at ~30 Hz
        self.timer = self.create_timer(
            1.0 / 30.0,
            self.publish_frame
        )

    def publish_frame(self):

        ret, frame = self.cap.read()

        if not ret:
            self.get_logger().warning(
                'Failed to capture frame'
            )
            return

        image_msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8'
        )

        image_msg.header.stamp = self.get_clock().now().to_msg()
        image_msg.header.frame_id = 'camera_frame'

        self.publisher.publish(image_msg)

    def destroy_node(self):

        if self.cap.isOpened():
            self.cap.release()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = CameraNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()