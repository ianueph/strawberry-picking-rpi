import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    ## imx708_wide will be intended to be used for depth estimation and object detection
    ## image preprocessing pipeline for the imx708_wide
    depth_camera_config = os.path.join(
                            get_package_share_directory("camera_ros"),
                            "config",
                            "depth_camera_config.yaml"
                        )
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("camera_ros"),
                "launch",
                "camera_node.launch.py"
            ),
        ),
        launch_arguments={
            "namespace": "depth_camera",
            "params_file": depth_camera_config
        }.items()
    )
    depth_camera_throttle = Node(
        package="topic_tools",
        executable="throttle",
        name="depth_camera_throttle",
        output="screen",
        arguments=[
            "messages",
            "/depth_camera/image_raw",
            "0.2",
            "/depth_camera/image_raw_throttle"
        ]
    )
    depth_camera_image_rect = Node(
        package="image_proc",
        executable="rectify_node",
        name="depth_camera_rectification",
        output="screen",
        remappings=[
            ("image", "/depth_camera/image_raw_throttle"),
            ("camera_info", "/depth_camera/camera_info"),
            ("image_rect", "/depth_camera/image_rect")
        ]
    )

    ## initialize depth estimation node
    ## depth estimation pipeline
    depth_anything_config = os.path.join(get_package_share_directory("depth_anything_v2_ros2"), 'config', 'params.yaml')
    depth_anything_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("depth_anything_v2_ros2"),
                "launch",
                "default.launch.py"
            ),
        ),
        launch_arguments={
            "params_file": depth_anything_config
        }.items()
    )
    depth_to_pointcloud = Node(
        package="depth_image_proc",
        executable="point_cloud_xyz_node",
        output="screen",
        remappings=[
            ("image_rect", "/depth_camera/depth/image_raw"),
            ("camera_info", "/depth_camera/depth/camera_info")
        ]
    )
    return LaunchDescription([
        depth_camera_launch,
        depth_camera_throttle,
        depth_camera_image_rect,
        depth_anything_launch,
        depth_to_pointcloud,
    ])