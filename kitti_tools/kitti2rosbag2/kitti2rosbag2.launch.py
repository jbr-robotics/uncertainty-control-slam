from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import os

package_name = 'kitti2rosbag2'

def generate_launch_description():
    # Define command-line arguments for the launch file
    sequence_arg = DeclareLaunchArgument(
        'sequence', default_value='0', description='KITTI sequence number'
    )
    data_dir_arg = DeclareLaunchArgument(
        'data_dir', default_value='/data/dataset', description='Path to KITTI dataset directory'
    )
    odom_dir_arg = DeclareLaunchArgument(
        'odom_dir', default_value='/data/odometry', description='Path to KITTI odometry directory'
    )
    bag_dir_arg = DeclareLaunchArgument(
        'bag_dir', default_value='/data/output', description='Output path for ROS2 bag files'
    )

    # Get launch configurations from the arguments
    sequence = LaunchConfiguration('sequence')
    data_dir = LaunchConfiguration('data_dir')
    odom_dir = LaunchConfiguration('odom_dir')
    bag_dir = LaunchConfiguration('bag_dir')

    # Dynamically configure the node's parameters
    kitti_rec = Node(
        package=package_name,
        namespace='',
        executable='kitti_rec_node',
        name='kitti_rec',
        parameters=[{
            'sequence': sequence,
            'data_dir': data_dir,
            'odom_dir': odom_dir,
            'bag_dir': bag_dir,
            'odom': True,
        }],
        output='screen'
    )

    return LaunchDescription([
        sequence_arg,
        data_dir_arg,
        odom_dir_arg,
        bag_dir_arg,
        kitti_rec,
    ])
