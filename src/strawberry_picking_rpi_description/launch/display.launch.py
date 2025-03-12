from launch import LaunchDescription
from launch.action import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():
    ld = LaunchDescription()
    
    default_model_path = PathJoinSubstitution([
        FindPackageShare('strawberry_picking_rpi_description'),
        'urdf', 
        'strawberry_picking_rpi.urdf.xacro'
        ])    
    
    xacro_to_urdf = xacro.process(default_model_path)
    
    
    
    return ld