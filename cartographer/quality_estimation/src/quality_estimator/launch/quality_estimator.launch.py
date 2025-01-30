from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    text_arg = DeclareLaunchArgument(
        'text',
        default_value='Hello, ROS2!',
        description='Text to publish'
    )
    config = os.path.join(
        get_package_share_directory('publisher'),
        'config',
        'publisher.yaml'
    )

    return LaunchDescription([
        text_arg,

        Node(
            package='publisher',
            executable='publisher',
            name='publisher',
            output='screen',
            parameters=[config,
                        {'text': LaunchConfiguration('text')}]
        ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            output='screen',
            # parameters=[{'video_device': '/dev/video0'}],  # Adjust if your webcam is on a different device
            remappings=[
                ('/image_raw', '/image_raw_sent'),
                ('/image_raw/compressed', '/camera/image_raw/compressed'),
                ('/image_raw/compressedDepth', '/camera/image_raw/compressedDepth'),
                ('/image_raw/theora', '/camera/image_raw/theora'),
            ]
        ),
    ])
