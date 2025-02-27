#!/usr/bin/env python3

import rclpy
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node as LaunchNode

from quality_estimator.launchers.base import BaseLauncher

class PbstreamToPgmLauncher(BaseLauncher):
    """
    Launcher for converting Cartographer's pbstream files to PGM map files.
    
    This launcher wraps the cartographer_pbstream_to_ros_map node, which converts
    a pbstream file (serialized Cartographer state) into a ROS map (.pgm and .yaml files).
    """
    
    # Register parameters using the class method
    BaseLauncher.register_parameter(
        "pbstream_filename",
        str,
        required=True,
        help="Filename of a pbstream to draw a map from."
    )
    
    BaseLauncher.register_parameter(
        "map_filestem",
        str,
        required=False,
        default="map",
        help="Stem of the output files (.pgm and .yaml)."
    )
    
    BaseLauncher.register_parameter(
        "resolution",
        float,
        required=False,
        default=0.05,
        help="Resolution of a grid cell in the drawn map in meters per pixel."
    )
    
    def generate_launch_description(self) -> LaunchDescription:
        """Generate launch description for the pbstream to pgm conversion."""
        
        # Create pbstream_to_ros_map node
        pbstream_to_pgm_node = LaunchNode(
            package='cartographer_ros',
            executable='cartographer_pbstream_to_ros_map',
            name='cartographer_pbstream_to_ros_map',
            arguments=[
                '-pbstream_filename', self.get_launch_configuration('pbstream_filename'),
                '-map_filestem', self.get_launch_configuration('map_filestem'),
                '-resolution', self.get_launch_configuration('resolution'),
            ],
            output='screen'
        )
        
        # Build launch description with base elements and node
        description = self.get_base_description_elements() + [
            pbstream_to_pgm_node
        ]
        
        return LaunchDescription(description)

# Generate main function
main = PbstreamToPgmLauncher.generate_main()

if __name__ == '__main__':
    main()
