from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("strawberry_picking_rpi", package_name="strawberry_picking_rpi_moveit_config").to_moveit_configs()
    return generate_rsp_launch(moveit_config)
