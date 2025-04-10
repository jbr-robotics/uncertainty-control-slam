#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode

from cartographer_tuner.tools.ros.base_ros_tool import BaseRosTool

__all__ = [
    "BagPlayLauncher"
]

class BagPlayLauncher(BaseRosTool):
    """
    Launcher for playing ROS 2 bag files with customizable parameters.
    """

    @classmethod
    def _register_params(cls):
        cls.register_parameter(
            "bag_path",
            str,
            required=True,
            help="Path to the ROS 2 bag file to play"
        )
        
        cls.register_parameter(
            "rate",
            float,
            required=False,
            default=1.0,
            help="Rate at which to play back messages. Valid range > 0.0"
        )
        
        cls.register_parameter(
            "start_offset",
            float,
            required=False,
            default=0.0,
            help="Start the playback player this many seconds into the bag file"
        )
    
    def generate_launch_description(self) -> LaunchDescription:
        arguments = ['play', self.get_launch_configuration('bag_path')]
        
        arguments.extend(['-r', str(self._rate)])
        
        arguments.extend(['--start-offset', str(self._start_offset)])
        
        # Always add clock flag without value
        arguments.append('--clock')
        
        # Always disable keyboard controls
        arguments.append('--disable-keyboard-controls')
        
        bag_play_node = LaunchNode(
            package='ros2bag',
            executable='ros2',
            name='bag_play',
            arguments=arguments,
            output='screen'
        )
        
        description = self.get_launch_arguments() + [
            bag_play_node
        ]
        
        return LaunchDescription(description) 