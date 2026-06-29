#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseArray
from vision_msgs.msg import Detection2DArray

from message_filters import Subscriber
from message_filters import ApproximateTimeSynchronizer


# --------------------------------------------------
# Camera Parameters
# --------------------------------------------------

IMAGE_WIDTH     = 640
IMAGE_HEIGHT    = 480
CAMERA_HFOV     = 49.0
CAMERA_FOV_HALF = CAMERA_HFOV / 2.0   # ±24.5°


class FusionNode(Node):

    def __init__(self):

        super().__init__('fusion_node')

        self.camera_sub     = Subscriber(self, Image,          '/camera/image_raw')
        self.detections_sub = Subscriber(self, Detection2DArray, '/detections')
        self.radar_sub      = Subscriber(self, PoseArray,      '/radar_targets')
        self.lidar_sub      = Subscriber(self, Range,          '/lidar_range')

        self.sync = ApproximateTimeSynchronizer(
            [self.camera_sub, self.detections_sub, self.radar_sub, self.lidar_sub],
            queue_size=200,
            slop=1.0
        )

        self.sync.registerCallback(self.fusion_callback)

        self.get_logger().info('Fusion Node Started (Synchronized)')

    def radar_bearing(self, pose):
        # Radar reports x as lateral offset and y as forward distance.
        forward  = pose.position.y
        lateral  = pose.position.x
        return math.degrees(math.atan2(lateral, forward))

    def camera_bearing(self, cx):
        pixel_offset = cx - IMAGE_WIDTH / 2.0
        return (pixel_offset / (IMAGE_WIDTH / 2.0)) * CAMERA_FOV_HALF

    def fusion_callback(self, image_msg, detections_msg, radar_msg, lidar_msg):

        print("\n" + "=" * 70)
        print("SYNCHRONIZED SENSOR FRAME")
        print("=" * 70)

        # --------------------------------------------------
        # Camera
        # --------------------------------------------------

        print("\nCamera")
        print(f"  Frame ID       : {image_msg.header.frame_id}")
        print(f"  Resolution     : {IMAGE_WIDTH} x {IMAGE_HEIGHT}")
        print(f"  Horizontal FOV : {CAMERA_HFOV}°")

        # --------------------------------------------------
        # Radar
        # --------------------------------------------------

        print("\nRadar")
        print(f"  Targets : {len(radar_msg.poses)}")

        for i, pose in enumerate(radar_msg.poses):

            x        = pose.position.x
            y        = pose.position.y
            distance = math.sqrt(x * x + y * y)
            bearing  = self.radar_bearing(pose)
            in_fov   = abs(bearing) <= CAMERA_FOV_HALF

            print(f"\n  Target {i + 1}")
            print(f"    x        : {x:.2f} m")
            print(f"    y        : {y:.2f} m")
            print(f"    Distance : {distance:.2f} m")
            print(f"    Angle    : {bearing:.1f}°")

            if not in_fov:
                print(f"    Status   : Outside Camera FOV")

        # --------------------------------------------------
        # LiDAR
        # --------------------------------------------------

        print("\nLiDAR")
        print(f"  Range : {lidar_msg.range:.2f} m")

        # --------------------------------------------------
        # YOLO
        # --------------------------------------------------

        print("\nYOLO")

        person_count = 0

        for detection in detections_msg.detections:

            if not detection.results:
                continue

            result     = detection.results[0]
            label      = result.hypothesis.class_id
            confidence = result.hypothesis.score
            cx         = detection.bbox.center.position.x
            cy         = detection.bbox.center.position.y
            width      = detection.bbox.size_x
            height     = detection.bbox.size_y
            cam_angle  = self.camera_bearing(cx)

            if label.lower() != 'person':
                continue

            person_count += 1

            print(f"\n  Person {person_count}")
            print(f"    Confidence : {confidence:.2f}")
            print(f"    Center     : ({cx:.1f}, {cy:.1f})")
            print(f"    Size       : ({width:.1f}, {height:.1f})")
            print(f"    Angle      : {cam_angle:.1f}°")

        print(f"\n  Persons : {person_count}")

        print("\n" + "=" * 70)

        self.get_logger().info(
            f"Radar={len(radar_msg.poses)} | "
            f"Persons={person_count} | "
            f"LiDAR={lidar_msg.range:.2f} m"
        )


def main(args=None):

    rclpy.init(args=args)
    node = FusionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()