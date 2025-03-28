#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode

from cartographer_tuner.tools.ros.base_ros_tool import BaseRosTool

__all__ = [
    "PbstreamToPgmLauncher"
]

class PbstreamToPgmLauncher(BaseRosTool):
    """
    Launcher for converting Cartographer's pbstream files to PGM map files.
    """

    @classmethod
    def _register_params(cls):
        cls.register_parameter(
            "pbstream_filename",
            str,
            required=True,
            help="Filename of a pbstream to draw a map from."
        )
        
        cls.register_parameter(
            "map_filestem",
            str,
            required=False,
            default="map",
            help="Stem of the output files (.pgm and .yaml)."
        )
        
        cls.register_parameter(
            "resolution",
            float,
            required=False,
            default=0.05,
            help="Resolution of a grid cell in the drawn map in meters per pixel."
        )
    
    def generate_launch_description(self) -> LaunchDescription:
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
        
        description = self.get_launch_arguments() + [
            pbstream_to_pgm_node
        ]
        
        return LaunchDescription(description)
