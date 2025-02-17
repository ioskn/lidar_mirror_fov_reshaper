import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory(
        'lidar_mirror_fov_reshaper_runtime'), 'config', 'params.yaml')    
    
    osg_runtime = Node(
        package='lidar_mirror_fov_reshaper_runtime',
        executable='lidar_mirror_fov_reshaper_runtime',
        name='lidar_mirror_fov_reshaper_runtime',
        parameters=[config],
        output='screen')
    
    # fake localization; Used for demo purposes only
    static_map_odom = Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0.0", "0.0", "0", "0", "0", "0", "map", "odom"])
    static_odom_base_link = Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                arguments=["0.0", "0.0", "0", "0", "0", "0", "odom", "base_link"])
    
    ld = LaunchDescription()
    ld.add_action(osg_runtime)
    # ld.add_action(static_map_odom)
    # ld.add_action(static_odom_base_link)
    
    return ld
