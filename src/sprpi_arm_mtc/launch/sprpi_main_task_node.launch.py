from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import os
from ament_index_python import get_package_share_directory

def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("strawberry_picking_rpi"
                                         ).robot_description_semantic(Path("config") / "strawberry_picking_rpi.srdf").to_dict()
    mtc_dir = get_package_share_directory("sprpi_arm_mtc")
    mtc_config = os.path.join(mtc_dir, "config", "main_task_node_config.yaml")  

    # MTC Demo node
    pick_place_demo = Node(
        package="sprpi_arm_mtc",
        executable="main_task_node",
        output="screen",
        parameters=[    
            moveit_config,
            mtc_config,
        ],
    )

    return LaunchDescription([
        pick_place_demo
        ])