from typing import Optional
import argparse
from launch import LaunchService
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch import LaunchDescription

class CartographerAssetsWriterLauncher:
    """Class to handle exporting files using Cartographer's assets writer."""
    
    def __init__(self):
        self._configuration_directory: Optional[str] = None
        self._config_file: Optional[str] = None
        self._bag_filenames: Optional[str] = None
        self._pose_graph_filename: Optional[str] = None

    def set_configuration_directory(self, directory: str) -> "CartographerAssetsWriterLauncher":
        """Set the directory containing configuration files."""
        self._configuration_directory = directory
        return self

    def set_config_file(self, config_file: str) -> "CartographerAssetsWriterLauncher":
        """Set the assets writer configuration file."""
        self._config_file = config_file
        return self

    def set_bag_filenames(self, filenames: str) -> "CartographerAssetsWriterLauncher":
        """Set input bag files."""
        self._bag_filenames = filenames
        return self

    def set_pose_graph_filename(self, filename: str) -> "CartographerAssetsWriterLauncher":
        """Set pose graph file."""
        self._pose_graph_filename = filename
        return self

    def generate_launch_description(self) -> LaunchDescription:
        """Generate launch description for the assets writer."""
        if None in [
            self._configuration_directory,
            self._config_file,
            self._bag_filenames,
            self._pose_graph_filename
        ]:
            raise ValueError("All parameters must be set before generating the launch description.")

        # Declare launch arguments
        configuration_directory_arg = DeclareLaunchArgument(
            'configuration_directory',
            default_value=self._configuration_directory
        )
        
        configuration_basename_arg = DeclareLaunchArgument(
            'configuration_basename',
            default_value=self._config_file
        )
        
        bag_filenames_arg = DeclareLaunchArgument(
            'bag_filenames',
            default_value=self._bag_filenames
        )
        
        pose_graph_filename_arg = DeclareLaunchArgument(
            'pose_graph_filename',
            default_value=self._pose_graph_filename
        )

        # Create assets writer node
        assets_writer_node = Node(
            package='cartographer_ros',
            executable='cartographer_assets_writer',
            name='cartographer_assets_writer',
            arguments=[
                '-configuration_directory', LaunchConfiguration('configuration_directory'),
                '-configuration_basename', LaunchConfiguration('configuration_basename'),
                '-bag_filenames', LaunchConfiguration('bag_filenames'),
                '-pose_graph_filename', LaunchConfiguration('pose_graph_filename'),
            ],
            output='screen'
        )

        return LaunchDescription([
            configuration_directory_arg,
            configuration_basename_arg,
            bag_filenames_arg,
            pose_graph_filename_arg,
            assets_writer_node
        ])

    def run(self):
        """Execute the assets writer node."""
        launch_service = LaunchService()
        launch_service.include_launch_description(self.generate_launch_description())
        launch_service.run()

def main():
    parser = argparse.ArgumentParser(description="Export files using Cartographer's assets writer")
    
    parser.add_argument("--configuration_directory", required=True,
                      help="Directory containing configuration files")
    parser.add_argument("--config_file", required=True,
                      help="Assets writer configuration file (e.g., assets_writer_ply.lua)")
    parser.add_argument("--bag_filenames", required=True,
                      help="Path to input bag file(s)")
    parser.add_argument("--pose_graph_filename", required=True,
                      help="Path to pose graph file (.pbstream)")

    args = parser.parse_args()

    launcher = (CartographerAssetsWriterLauncher()
        .set_configuration_directory(args.configuration_directory)
        .set_config_file(args.config_file)
        .set_bag_filenames(args.bag_filenames)
        .set_pose_graph_filename(args.pose_graph_filename))

    launcher.run()

if __name__ == "__main__":
    main()
