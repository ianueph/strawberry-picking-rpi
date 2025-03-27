from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "strawberry_picking_rpi", package_name="strawberry_picking_rpi_moveit_config"
        ).joint_limits(file_path="config/joint_limits.yaml"
                       ).to_moveit_configs()
    return generate_demo_launch(moveit_config)
    
