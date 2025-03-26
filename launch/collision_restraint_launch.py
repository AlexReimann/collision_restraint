from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    default_params = get_package_share_directory('collision_restraint') + '/config/default.yaml'
    
    return LaunchDescription([
        Node(
            package='collision_restraint',
            namespace='collision_restraint',
            executable='collision_restraint_node',
            name='collision_restraint_node',
            parameters=[default_params]
        )
    ])
