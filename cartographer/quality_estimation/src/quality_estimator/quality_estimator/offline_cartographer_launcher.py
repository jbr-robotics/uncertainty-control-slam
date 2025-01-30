from typing import Optional
import argparse
import rclpy
from rclpy.node import Node
from launch import LaunchService
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node as LaunchNode
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch_ros.actions import SetRemap

class OfflineCartographerLauncher:
    def __init__(self):
        self._skip_seconds: Optional[int] = None
        self._no_rviz: Optional[str] = None
        self._bag_filenames: Optional[str] = None
        self._rviz_config: Optional[str] = None
        self._configuration_directory: Optional[str] = None
        self._configuration_basenames: Optional[str] = None
        self._points2_topic: Optional[str] = None
        self._imu_topic: Optional[str] = None
        self._save_state_filename: Optional[str] = None

    def set_skip_seconds(self, seconds: int) -> "OfflineCartographerLauncher":
        self._skip_seconds = seconds
        return self

    def set_no_rviz(self, no_rviz: str) -> "OfflineCartographerLauncher":
        self._no_rviz = no_rviz
        return self

    def set_bag_filenames(self, filenames: str) -> "OfflineCartographerLauncher":
        self._bag_filenames = filenames
        return self

    def set_rviz_config(self, config_path: str) -> "OfflineCartographerLauncher":
        self._rviz_config = config_path
        return self

    def set_configuration_directory(self, directory: str) -> "OfflineCartographerLauncher":
        self._configuration_directory = directory
        return self

    def set_configuration_basenames(self, basenames: str) -> "OfflineCartographerLauncher":
        self._configuration_basenames = basenames
        return self

    def set_points2_topic(self, topic: str) -> "OfflineCartographerLauncher":
        self._points2_topic = topic
        return self

    def set_imu_topic(self, topic: str) -> "OfflineCartographerLauncher":
        self._imu_topic = topic
        return self

    def set_save_state_filename(self, filename: str) -> "OfflineCartographerLauncher":
        self._save_state_filename = filename
        return self

    def generate_launch_description(self) -> LaunchDescription:
        if None in [
            self._bag_filenames, self._configuration_directory,
            self._configuration_basenames, self._points2_topic, self._imu_topic, self._save_state_filename
        ]:
            raise ValueError("All parameters must be set before generating the launch description.")

        # Declare Launch Arguments
        bag_filenames_arg = DeclareLaunchArgument('bag_filenames', default_value=self._bag_filenames)
        no_rviz_arg = DeclareLaunchArgument('no_rviz', default_value=self._no_rviz)
        rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value=self._rviz_config)
        configuration_directory_arg = DeclareLaunchArgument('configuration_directory', default_value=self._configuration_directory)
        configuration_basenames_arg = DeclareLaunchArgument('configuration_basenames', default_value=self._configuration_basenames)
        skip_seconds_arg = DeclareLaunchArgument('skip_seconds', default_value=str(self._skip_seconds))
        save_state_filename_arg = DeclareLaunchArgument('save_state_filename', default_value=self._save_state_filename)

        # Nodes
        if self._no_rviz != 'false':
            rviz_node = None
        else:
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

        cartographer_offline_node_node = LaunchNode(
            package='cartographer_ros',
            executable='cartographer_offline_node',
            parameters=[{'use_sim_time': True}],
            on_exit=Shutdown(),
            arguments=[
                '-configuration_directory', LaunchConfiguration('configuration_directory'),
                '-configuration_basenames', LaunchConfiguration('configuration_basenames'),
                '-bag_filenames', LaunchConfiguration('bag_filenames'),
                '-skip_seconds', LaunchConfiguration('skip_seconds'),
                '-save_state_filename', LaunchConfiguration('save_state_filename'),
            ],
            output='screen',
        )

        # Set Remapping
        set_remap1 = SetRemap(self._points2_topic, '/points2')
        set_remap2 = SetRemap(self._imu_topic, '/imu')

        # Return the launch description
        description = [
            bag_filenames_arg,
            no_rviz_arg,
            configuration_directory_arg,
            configuration_basenames_arg,
            skip_seconds_arg,
            save_state_filename_arg,

            set_remap1,
            set_remap2,

            cartographer_occupancy_grid_node,
            cartographer_offline_node_node,
        ]

        if rviz_node is not None:
            description += [rviz_node, rviz_config_arg]

        return LaunchDescription(description)

    def run(self):
        # Create LaunchService and run the description
        launch_service = LaunchService()
        launch_service.include_launch_description(self.generate_launch_description())
        launch_service.run()

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description='Offline Cartographer Launcher')
    parser.add_argument('--skip_seconds', type=int, required=True, help='Seconds to skip from the bag file')
    parser.add_argument('--no_rviz', type=str, required=True, help='Disable RViz')
    parser.add_argument('--bag_filenames', type=str, required=True, help='Path to the bag file')
    parser.add_argument('--rviz_config', type=str, help='Path to the RViz configuration file')
    parser.add_argument('--configuration_directory', type=str, required=True, help='Directory of configuration files')
    parser.add_argument('--configuration_basenames', type=str, required=True, help='Configuration file basenames')
    parser.add_argument('--points2_topic', type=str, required=True, help='Points2 topic')
    parser.add_argument('--imu_topic', type=str, required=True, help='IMU topic')
    parser.add_argument('--save_state_filename', type=str, required=True, help='Explicit name of the file to which the serialized state will be written before shutdown')

    parsed_args = parser.parse_args()

    if parsed_args.no_rviz != 'false':
        parser.error("only --no_rviz=true is supported now")
    # Check if `rviz_config` is required
    if (parsed_args.no_rviz != 'true') and (parsed_args.rviz_config is None):
        parser.error("--rviz_config is required when --no_rviz is set to False.")


    launcher = (
        OfflineCartographerLauncher()
        .set_skip_seconds(parsed_args.skip_seconds)
        .set_no_rviz(parsed_args.no_rviz)
        .set_bag_filenames(parsed_args.bag_filenames)
        .set_rviz_config(parsed_args.rviz_config)
        .set_configuration_directory(parsed_args.configuration_directory)
        .set_configuration_basenames(parsed_args.configuration_basenames)
        .set_points2_topic(parsed_args.points2_topic)
        .set_imu_topic(parsed_args.imu_topic)
        .set_save_state_filename(parsed_args.save_state_filename)
    )
    launcher.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
