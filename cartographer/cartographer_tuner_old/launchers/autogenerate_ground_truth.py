from typing import Optional
import subprocess
from quality_estimator.launchers.base import BaseLauncher
from launch import LaunchDescription

class AutogenerateGroundTruthLauncher(BaseLauncher):
    # TODO: this class is not tested
    """Launch Cartographer's ground truth autogeneration tool."""

    # Register all parameters
    BaseLauncher.register_parameter(
        "pose_graph_filename",
        str,
        required=True,
        help="Path to the pose graph file (.pbstream)"
    )
    BaseLauncher.register_parameter(
        "output_filename",
        str,
        required=True,
        help="Path to the output relations file"
    )
    BaseLauncher.register_parameter(
        "min_covered_distance",
        float,
        required=True,
        help="Minimum covered distance for ground truth generation"
    )
    BaseLauncher.register_parameter(
        "outlier_threshold_meters",
        float,
        required=True,
        help="Outlier threshold in meters for ground truth generation"
    )
    BaseLauncher.register_parameter(
        "outlier_threshold_radians",
        float,
        required=True,
        help="Outlier threshold in radians for ground truth generation"
    )

    def generate_launch_description(self) -> LaunchDescription:
        """This launcher doesn't use ROS2 launch system, so this method is not used."""
        return LaunchDescription([])

    def run(self) -> None:
        """Execute the ground truth autogeneration command.
        
        This overrides the base class run() method to execute a direct command
        instead of using the ROS2 launch system.
        """
        command = [
            "cartographer_autogenerate_ground_truth",
            "-pose_graph_filename", self._pose_graph_filename,
            "-output_filename", self._output_filename,
            "-min_covered_distance", str(self._min_covered_distance),
            "-outlier_threshold_meters", str(self._outlier_threshold_meters),
            "-outlier_threshold_radians", str(self._outlier_threshold_radians)
        ]

        try:
            subprocess.run(command, check=True)
            print("cartographer_autogenerate_ground_truth finished successfully")
        except subprocess.CalledProcessError as e:
            print("cartographer_autogenerate_ground_truth failed with error:", e)
            raise

# Generate main function
main = AutogenerateGroundTruthLauncher.generate_main()

if __name__ == '__main__':
    main()
