from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_yolo_node(context):
    # Optional: run YOLO with a separate Python interpreter (e.g. a venv
    # with ultralytics/PyTorch), keeping the system Python lean for ROS 2.
    #   ros2 launch radar_lidar_fusion sensors.launch.py \
    #       yolo_python:=$HOME/venvs/yolo_env/bin/python3
    yolo_python = LaunchConfiguration('yolo_python').perform(context)

    return [Node(
        package='radar_lidar_fusion',
        executable='yolo_node',
        name='yolo_node',
        output='screen',
        prefix=(yolo_python + ' ') if yolo_python else None
    )]


def generate_launch_description():

    yolo_python_arg = DeclareLaunchArgument(
        'yolo_python',
        default_value='',
        description='Python interpreter for yolo_node (empty = system Python)'
    )

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

    yolo_node = OpaqueFunction(function=make_yolo_node)

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

        yolo_python_arg,

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