from launch.actions import Shutdown
from launch_ros.actions import Node as LaunchNode
from launch import LaunchDescription

from cartographer_tuner.tools.ros.base_ros_tool import BaseRosTool
from cartographer_tuner.tools.exceptions import ExternalToolParameterException  

__all__ = [
    "OfflineCartographerLauncher"
]

class OfflineCartographerLauncher(BaseRosTool):
    """Launch offline Cartographer SLAM with specified parameters."""

    @classmethod
    def _register_params(cls):
        cls.register_parameter(
            "skip_seconds",
            int,
            required=False,
            default=0,
            help="Seconds to skip from the bag file"
        )
        cls.register_parameter(
            "no_rviz",
            str,
            required=False,
            help="Disable RViz visualization",
            default='false'
        )
        cls.register_parameter(
            "bag_filename",
            str,
            required=True,
            help="Path to the bag file"
        )
        cls.register_parameter(
            "rviz_config",
            str,
            required=False,
            help="Path to the RViz configuration file",
            default=None
        )
        cls.register_parameter(
            "configuration_directory",
            str,
            required=True,
            help="Directory of configuration files"
        )
        cls.register_parameter(
            "configuration_basename",
            str,
            required=True,
            help="Configuration file basenames"
        )
        cls.register_parameter(
            "save_state_filename",
            str,
            required=True,
            help="Path to save the final state"
        )

    def generate_launch_description(self) -> LaunchDescription:
        if self._no_rviz != 'false':
            rviz_node = None
        else:
            if self._rviz_config is None:
                raise ExternalToolParameterException("RViz configuration file is required when no_rviz is false")
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
                '-configuration_basenames', self.get_launch_configuration('configuration_basename'),
                '-bag_filenames', self.get_launch_configuration('bag_filename'),
                '-skip_seconds', self.get_launch_configuration('skip_seconds'),
                '-save_state_filename', self.get_launch_configuration('save_state_filename'),
            ],
            output='screen',
        )

        description = self.get_launch_arguments() + [
            cartographer_occupancy_grid_node,
            cartographer_offline_node,
        ]

        if rviz_node is not None:
            description.append(rviz_node)

        return LaunchDescription(description)

