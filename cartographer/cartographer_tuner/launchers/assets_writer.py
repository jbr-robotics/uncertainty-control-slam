from typing import Optional
import rclpy
from launch import LaunchService, LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node as LaunchNode

from quality_estimator.launchers.base import BaseLauncher

class AssetsWriterLauncher(BaseLauncher):
    # TODO: this class is not tested
    """Launch Cartographer's assets writer to export files."""

    # Register all parameters
    BaseLauncher.register_parameter(
        "configuration_directory",
        str,
        required=True,
        help="Directory containing configuration files"
    )
    BaseLauncher.register_parameter(
        "configuration_basename",
        str,
        required=True,
        help="Assets writer configuration file (e.g., assets_writer_ply.lua)"
    )
    BaseLauncher.register_parameter(
        "bag_filenames",
        str,
        required=True,
        help="Path to input bag file(s)"
    )
    BaseLauncher.register_parameter(
        "pose_graph_filename",
        str,
        required=True,
        help="Path to pose graph file (.pbstream)"
    )

    def generate_launch_description(self) -> LaunchDescription:
        """Generate launch description for the assets writer."""
        
        assets_writer_node = LaunchNode(
            package='cartographer_ros',
            executable='cartographer_assets_writer',
            name='cartographer_assets_writer',
            arguments=[
                '-configuration_directory', self.get_launch_configuration('configuration_directory'),
                '-configuration_basename', self.get_launch_configuration('configuration_basename'),
                '-bag_filenames', self.get_launch_configuration('bag_filenames'),
                '-pose_graph_filename', self.get_launch_configuration('pose_graph_filename'),
            ],
            output='screen'
        )

        # Build launch description with base elements and node
        description = self.get_base_description_elements() + [
            assets_writer_node
        ]

        return LaunchDescription(description)

# Generate main function
main = AssetsWriterLauncher.generate_main()

if __name__ == '__main__':
    main()
