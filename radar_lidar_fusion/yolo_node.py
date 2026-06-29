import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from ultralytics import YOLO

from vision_msgs.msg import (
    Detection2DArray,
    Detection2D,
    BoundingBox2D,
    ObjectHypothesisWithPose
)


class YoloNode(Node):

    def __init__(self):
        super().__init__('yolo_node')

        self.bridge = CvBridge()

        # Model
        self.model_path = "yolov8n.pt"
        self.model = YOLO(self.model_path)

        # Confidence threshold
        self.confidence_threshold = 0.5

        # Prevent multiple simultaneous inferences
        self.processing = False

        # Subscribe to camera images
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publish annotated image
        self.detections_image_publisher = self.create_publisher(
            Image,
            '/camera/detections_image',
            10
        )

        # Publish Detection2DArray
        self.detections_publisher = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )

        self.get_logger().info(
            f'YOLO node started ({self.model_path})'
        )

    def image_callback(self, msg):

        if self.processing:
            return

        self.processing = True

        try:

            # Convert ROS image to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            # Run YOLO
            results = self.model(
                frame,
                conf=self.confidence_threshold,
                verbose=False
            )

            # --------------------------------------------------
            # Publish annotated image
            # --------------------------------------------------

            annotated_frame = results[0].plot()

            image_msg = self.bridge.cv2_to_imgmsg(
                annotated_frame,
                encoding='bgr8'
            )

            image_msg.header = msg.header

            self.detections_image_publisher.publish(
                image_msg
            )

            # --------------------------------------------------
            # Publish Detection2DArray
            # --------------------------------------------------

            detection_array = Detection2DArray()
            detection_array.header = msg.header

            detection_count = 0

            for result in results:

                for box in result.boxes:

                    confidence = float(box.conf[0])

                    if confidence < self.confidence_threshold:
                        continue

                    cls_id = int(box.cls[0])
                    label = self.model.names[cls_id]

                    x1, y1, x2, y2 = box.xyxy[0].tolist()

                    detection = Detection2D()
                    detection.header = msg.header

                    bbox = BoundingBox2D()

                    bbox.center.position.x = (x1 + x2) / 2.0
                    bbox.center.position.y = (y1 + y2) / 2.0

                    bbox.size_x = x2 - x1
                    bbox.size_y = y2 - y1

                    detection.bbox = bbox

                    hypothesis = ObjectHypothesisWithPose()

                    hypothesis.hypothesis.class_id = label
                    hypothesis.hypothesis.score = confidence

                    detection.results.append(
                        hypothesis
                    )

                    detection_array.detections.append(
                        detection
                    )

                    detection_count += 1

                    self.get_logger().debug(
                        f'{label}: {confidence:.2f}'
                    )

            self.detections_publisher.publish(
                detection_array
            )

            self.get_logger().info(
                f'Published {detection_count} detection(s)'
            )

        except Exception as e:

            self.get_logger().error(
                f'YOLO Error: {e}'
            )

        finally:

            self.processing = False

    def destroy_node(self):

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = YoloNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()