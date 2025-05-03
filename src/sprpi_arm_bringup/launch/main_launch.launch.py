import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    ## declare params
    image_frequency = LaunchConfiguration("image_frequency")
    debug = LaunchConfiguration("debug")
    declare_image_frequency_arg = DeclareLaunchArgument(
        "image_frequency",
        default_value="0.2",
        description="The max frequency (hz) that the system will process images"
    )
    declare_debug_arg = DeclareLaunchArgument(
        "debug",
        default_value="True",
        description="Will the launch file bring up the yolo debug nodes (True, False)"
    )
    
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
        output="log",
        arguments=[
            "messages",
            "/depth_camera/image_raw",
            image_frequency,
            "/depth_camera/image_raw_throttle",
            '--ros-args', '--log-level', 'error'
        ]
    )
    depth_camera_image_rect = Node(
        package="image_proc",
        executable="rectify_node",
        name="depth_camera_rectification",
        output="log",
        remappings=[
            ("image", "/depth_camera/image_raw_throttle"),
            ("camera_info", "/depth_camera/camera_info"),
            ("image_rect", "/depth_camera/image_rect")
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )
    strawberry_object_tracking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("yolo_bringup"),
                "launch",
                "yolov8.launch.py"
            )
        ),
        launch_arguments={
            "model": os.path.join(
                get_package_share_directory("yolo_bringup"),
                "launch",
                "strawberry_object_tracking.pt"
            ),
            "input_image_topic": "/depth_camera/image_rect",
            "target_frame": "depth_camera",
            "image_reliability": "2",
            "depth_image_reliability": "2",
            "depth_info_reliability": "2",
            "depth_image_units_divisor": "1",
            "input_depth_info_topic": "/depth_camera/depth/camera_info",
            "input_depth_topic": "/depth_camera/depth/image_raw",
            "use_3d": "True",
            "use_debug": debug,
            "device": "cpu",
            "namespace": "object_detection",
        }.items()
    )
    
    ## imx708 will be intended to be used for image classification
    ## image preprocessing pipeline for the imx708
    mirror_camera_config = os.path.join(
                            get_package_share_directory("camera_ros"),
                            "config",
                            "mirror_camera_config.yaml"
                        )
    mirror_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("camera_ros"),
                "launch",
                "camera_node.launch.py"
            ),
        ),
        launch_arguments={
            "namespace": "mirror_camera",
            "params_file": mirror_camera_config
        }.items()
    )
    mirror_camera_throttle = Node(
        package="topic_tools",
        executable="throttle",
        name="mirror_camera_throttle",
        output="log",
        arguments=[
            "messages",
            "/mirror_camera/image_raw",
            image_frequency,
            "/mirror_camera/image_raw_throttle",
            '--ros-args', '--log-level', 'error'
        ]
    )
    mirror_camera_image_rect = Node(
        package="image_proc",
        executable="rectify_node",
        name="mirror_camera_rectification",
        output="log",
        remappings=[
            ("image", "/mirror_camera/image_raw_throttle"),
            ("camera_info", "/mirror_camera/camera_info"),
            ("image_rect", "/mirror_camera/image_rect")
        ],
        arguments=['--ros-args', '--log-level', 'error']
    )
    healthy_diseased_classification_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("yolo_bringup"),
                "launch",
                "yolov8.launch.py"
            )
        ),
        launch_arguments={
            "model": os.path.join(
                get_package_share_directory("yolo_bringup"),
                "launch",
                "strawberry_image_class.pt"
            ),
            "input_image_topic": "/mirror_camera/image_rect",
            "image_reliability": "2",
            "device": "cpu",
            "use_debug": debug,
            "use_tracking": "False",
            "use_3d": "False",
            "namespace": "image_class",
        }.items()
    )
    mirror_camera_delay = TimerAction(
        period=6.0,
        actions=[
            mirror_camera_launch,
            mirror_camera_throttle,
            mirror_camera_image_rect, 
            healthy_diseased_classification_launch,
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
    
    ## visualize in rviz
    sprpi_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("strawberry_picking_rpi_description"),
                "launch",
                "display.launch.py"
            ),
        )
    )    
    
    ## MTC node for task planning
    MTC_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("sprpi_arm_mtc"),
                "launch",
                "sprpi_main_task_node.launch.py"
            ),
        )
    )
    
    return LaunchDescription([
        declare_image_frequency_arg,
        depth_camera_launch,
        depth_camera_throttle,
        depth_camera_image_rect,
        mirror_camera_delay,
        depth_anything_launch,
        depth_to_pointcloud,
        strawberry_object_tracking_launch,
        MTC_node_launch,
    ])