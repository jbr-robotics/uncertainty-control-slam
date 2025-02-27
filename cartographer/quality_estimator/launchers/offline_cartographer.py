from typing import Optional
import rclpy
from launch import LaunchService, LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node as LaunchNode
from launch_ros.actions import SetRemap

from quality_estimator.launchers.base import BaseLauncher

class OfflineCartographerLauncher(BaseLauncher):
    """Launch offline Cartographer SLAM with specified parameters."""

    # Register all parameters
    BaseLauncher.register_parameter(
        "skip_seconds",
        int,
        required=True,
        help="Seconds to skip from the bag file"
    )
    BaseLauncher.register_parameter(
        "no_rviz",
        str,
        required=True,
        help="Disable RViz visualization"
    )
    BaseLauncher.register_parameter(
        "bag_filenames",
        str,
        required=True,
        help="Path to the bag file"
    )
    BaseLauncher.register_parameter(
        "rviz_config",
        str,
        required=False,
        help="Path to the RViz configuration file",
        default=None
    )
    BaseLauncher.register_parameter(
        "configuration_directory",
        str,
        required=True,
        help="Directory of configuration files"
    )
    BaseLauncher.register_parameter(
        "configuration_basenames",
        str,
        required=True,
        help="Configuration file basenames"
    )
    BaseLauncher.register_parameter(
        "save_state_filename",
        str,
        required=True,
        help="Path to save the final state"
    )

    def generate_launch_description(self) -> LaunchDescription:
        """Generate launch description for offline Cartographer."""
        # Nodes
        if self._no_rviz != 'false':
            rviz_node = None
        else:
            assert self._rviz_config is not None, "RViz configuration file is required when no_rviz is false"
            rviz_node = LaunchNode(
                package='rviz2',
                executable='rviz2',
                on_exit=Shutdown(),
                arguments=['-d', self._rviz_config],
                parameters=[{'use_sim_time': True}],
            )
            
        cartographer_occupancy_grid_node = LaunchNode(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            parameters=[{'use_sim_time': True}, {'resolution': 0.05}],
        )

        cartographer_offline_node = LaunchNode(
            package='cartographer_ros',
            executable='cartographer_offline_node',
            parameters=[{'use_sim_time': True}],
            on_exit=Shutdown(),
            arguments=[
                '-configuration_directory', self.get_launch_configuration('configuration_directory'),
                '-configuration_basenames', self.get_launch_configuration('configuration_basenames'),
                '-bag_filenames', self.get_launch_configuration('bag_filenames'),
                '-skip_seconds', self.get_launch_configuration('skip_seconds'),
                '-save_state_filename', self.get_launch_configuration('save_state_filename'),
            ],
            output='screen',
        )

        # Build launch description with base elements and nodes
        description = self.get_base_description_elements() + [
            cartographer_occupancy_grid_node,
            cartographer_offline_node,
        ]

        if rviz_node is not None:
            description.append(rviz_node)

        return LaunchDescription(description)

# Generate main function
main = OfflineCartographerLauncher.generate_main()

if __name__ == '__main__':
    main()
