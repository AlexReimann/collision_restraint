from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    default_params = get_package_share_directory('collision_restraint') + '/config/default.yaml'
    override_params = get_package_share_directory('collision_restraint') + '/config/nav2_tb3_simulation_test.yaml'
    
    return LaunchDescription([
        Node(
            package='collision_restraint',
            namespace='collision_restraint',
            executable='collision_restraint_node',
            name='collision_restraint_node',
            parameters=[default_params, override_params],
            remappings=[
                ('output', '/cmd_vel'),
                ('sub_point_cloud', '/cloud'),
            ],
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='laserscan_to_pointcloud_node',
            name='laserscan_to_pointcloud',
            remappings=[('scan_in', '/scan'),
                        ('cloud', '/cloud'),
                        ],
            parameters=[{'target_frame': 'base_scan', 'transform_tolerance': 0.01}]
        ),
    ])
