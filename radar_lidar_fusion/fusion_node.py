#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseArray
from vision_msgs.msg import Detection2DArray

from message_filters import Subscriber
from message_filters import ApproximateTimeSynchronizer

import tf2_ros
import tf2_geometry_msgs

from geometry_msgs.msg import PointStamped

# --------------------------------------------------
# Camera Parameters
# --------------------------------------------------

IMAGE_WIDTH         = 640
IMAGE_HEIGHT        = 480
CAMERA_HFOV         = 49.0
CAMERA_FOV_HALF     = CAMERA_HFOV / 2.0   # ±24.5°
ANGLE_MATCH_THRESH  = 10.0                 # degrees


class FusionNode(Node):

    def __init__(self):

        super().__init__('fusion_node')

        # TF Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.camera_sub     = Subscriber(self, Image,           '/camera/image_raw')
        self.detections_sub = Subscriber(self, Detection2DArray, '/detections')
        self.radar_sub      = Subscriber(self, PoseArray,       '/radar_targets')
        self.lidar_sub      = Subscriber(self, Range,           '/lidar_range')

        self.sync = ApproximateTimeSynchronizer(
            [self.camera_sub, self.detections_sub, self.radar_sub, self.lidar_sub],
            queue_size=200,
            slop=1.0
        )

        self.sync.registerCallback(self.fusion_callback)

        self.get_logger().info('Fusion Node Started (Synchronized)')

    def radar_bearing(self, pose):
        # REP-103: x = forward, y = left
        x = pose.position.x
        y = pose.position.y
        return math.degrees(math.atan2(y, x))

    def camera_bearing(self, cx):
        pixel_offset = cx - IMAGE_WIDTH / 2.0
        return (pixel_offset / (IMAGE_WIDTH / 2.0)) * CAMERA_FOV_HALF
    
    def transform_radar_point(
            self,
            x,
            y,
            radar_frame,
            camera_frame,
            stamp):

        point = PointStamped()

        point.header.frame_id = radar_frame
        point.header.stamp = stamp

        point.point.x = x
        point.point.y = y
        point.point.z = 0.0

        try:

            transformed = self.tf_buffer.transform(
                point,
                camera_frame,
                timeout=Duration(seconds=0.2)
            )

            return (
                transformed.point.x,
                transformed.point.y,
                transformed.point.z
            )

        except Exception as e:

            self.get_logger().warn(
                f"TF failed : {e}"
            )

            return None

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
        # Radar — collect targets inside camera FOV
        # --------------------------------------------------

        print("\nRadar")
        print(f"  Targets : {len(radar_msg.poses)}")

        radar_targets = []

        for i, pose in enumerate(radar_msg.poses):

            raw_x = pose.position.x
            raw_y = pose.position.y

            camera_point = self.transform_radar_point(

                raw_x,
                raw_y,

                radar_msg.header.frame_id,

                image_msg.header.frame_id,

                radar_msg.header.stamp

            )

            if camera_point is None:
                continue

            cam_x, cam_y, cam_z = camera_point
            distance = math.sqrt(

                cam_x**2 +
                cam_y**2

            )

            bearing = math.degrees(

                math.atan2(
                    cam_y,
                    cam_x
                )

            )
            in_fov   = abs(bearing) <= CAMERA_FOV_HALF

            print(f"\n  Target {i + 1}")
            print(f"Radar x : {raw_x:.2f}")
            print(f"Radar y : {raw_y:.2f}")

            print(f"Camera x : {cam_x:.2f}")
            print(f"Camera y : {cam_y:.2f}")
            print(f"    Distance : {distance:.2f} m")
            print(f"    Angle    : {bearing:.1f}°")

            if not in_fov:
                print(f"    Status   : Outside Camera FOV")
                continue

            radar_targets.append({
                'id'      : i + 1,
                'distance': distance,
                'bearing' : bearing,
            })

        # --------------------------------------------------
        # LiDAR
        # --------------------------------------------------

        print("\nLiDAR")
        print(f"  Range : {lidar_msg.range:.2f} m")

        # --------------------------------------------------
        # YOLO — collect person detections
        # --------------------------------------------------

        print("\nYOLO")

        person_detections = []
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

            person_detections.append({
                'id'        : person_count,
                'confidence': confidence,
                'cam_angle' : cam_angle,
            })

        print(f"\n  Persons : {person_count}")

        # --------------------------------------------------
        # Fusion — match radar targets to YOLO persons
        # --------------------------------------------------

        print("\nFusion")

        if not radar_targets:
            print("  No radar targets inside camera FOV.")

        elif not person_detections:
            print("  No YOLO persons detected.")

        else:

            for rt in radar_targets:

                best_person = None
                best_diff   = float('inf')

                for pd in person_detections:

                    diff = abs(rt['bearing'] - pd['cam_angle'])

                    if diff < best_diff:
                        best_diff   = diff
                        best_person = pd

                if best_diff <= ANGLE_MATCH_THRESH:

                    print(
                        f"  MATCH  Radar T{rt['id']} "
                        f"(bearing={rt['bearing']:+.1f}°, dist={rt['distance']:.2f} m)  "
                        f"<->  Person {best_person['id']} "
                        f"(angle={best_person['cam_angle']:+.1f}°, "
                        f"conf={best_person['confidence']:.2f})  "
                        f"Δ={best_diff:.1f}°"
                    )

                else:

                    print(
                        f"  NO MATCH  Radar T{rt['id']} "
                        f"(bearing={rt['bearing']:+.1f}°)  "
                        f"closest Δ={best_diff:.1f}° "
                        f"(threshold={ANGLE_MATCH_THRESH}°)"
                    )

        print("\n" + "=" * 70)

        self.get_logger().info(
            f"Radar(FOV)={len(radar_targets)}/{len(radar_msg.poses)} | "
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


def __name__check():
    pass


if __name__ == '__main__':
    main()