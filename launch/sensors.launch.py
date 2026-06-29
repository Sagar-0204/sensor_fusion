from launch import LaunchDescription
from launch_ros.actions import Node

YOLO_PYTHON = "/home/rika/venvs/yolo_env/bin/python3"


def generate_launch_description():

    # --------------------------------------------------
    # Global TF
    # --------------------------------------------------

    map_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.0', '0.0', '0.0',      # x y z
            '0.0', '0.0', '0.0',      # roll pitch yaw
            'map',
            'base_link'
        ]
    )

    # --------------------------------------------------
    # Sensor TFs
    # base_link is located at the camera optical center
    # --------------------------------------------------

    base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.0', '0.0', '0.0',
            '0.0', '0.0', '0.0',
            'base_link',
            'camera_frame'
        ]
    )

    base_to_radar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.0', '0.015', '0.0',    # 1.5 cm left of camera
            '0.0', '0.0', '0.0',
            'base_link',
            'radar_frame'
        ]
    )

    base_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0.0', '0.0', '-0.030',   # 3 cm below camera
            '0.0', '0.0', '0.0',
            'base_link',
            'lidar_frame'
        ]
    )

    # --------------------------------------------------
    # Sensor Nodes
    # --------------------------------------------------

    camera_node = Node(
        package='radar_lidar_fusion',
        executable='camera_node',
        name='camera_node',
        output='screen'
    )

    radar_node = Node(
        package='radar_lidar_fusion',
        executable='radar_node',
        name='radar_node',
        output='screen'
    )

    lidar_node = Node(
        package='radar_lidar_fusion',
        executable='lidar_node',
        name='lidar_node',
        output='screen'
    )

    # --------------------------------------------------
    # Perception Nodes
    # --------------------------------------------------

    yolo_node = Node(
        package='radar_lidar_fusion',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        prefix=[YOLO_PYTHON + ' ']
    )

    fusion_node = Node(
        package='radar_lidar_fusion',
        executable='fusion_node',
        name='fusion_node',
        output='screen'
    )

    # --------------------------------------------------
    # Launch Description
    # --------------------------------------------------

    return LaunchDescription([

        # TF Tree
        map_to_base,
        base_to_camera,
        base_to_radar,
        base_to_lidar,

        # Sensors
        camera_node,
        radar_node,
        lidar_node,

        # Perception
        yolo_node,
        fusion_node,
    ])