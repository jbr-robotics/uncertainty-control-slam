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
import subprocess

class CartographerAutogenerateGroundTruthLauncher:
    def __init__(self):
        self._pose_graph_filename: Optional[str] = None
        self._output_filename: Optional[str] = None
        self._min_covered_distance: Optional[float] = None
        self._outlier_threshold_meters: Optional[float] = None
        self._outlier_threshold_radians: Optional[float] = None

    def set_pose_graph_filename(self, filename: str) -> "CartographerAutogenerateGroundTruthLauncher":
        self._pose_graph_filename = filename
        return self

    def set_output_filename(self, filename: str) -> "CartographerAutogenerateGroundTruthLauncher":
        self._output_filename = filename
        return self

    def set_min_covered_distance(self, distance: float) -> "CartographerAutogenerateGroundTruthLauncher":
        self._min_covered_distance = distance
        return self

    def set_outlier_threshold_meters(self, threshold: float) -> "CartographerAutogenerateGroundTruthLauncher":
        self._outlier_threshold_meters = threshold
        return self

    def set_outlier_threshold_radians(self, threshold: float) -> "CartographerAutogenerateGroundTruthLauncher":
        self._outlier_threshold_radians = threshold
        return self

    def run(self):
        if None in [
            self._pose_graph_filename,
            self._output_filename,
            self._min_covered_distance,
            self._outlier_threshold_meters,
            self._outlier_threshold_radians
        ]:
            raise ValueError("All parameters must be set before running the application.")

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
            # print("Command output:\n", result.stdout)
            print("Command finished")
        except subprocess.CalledProcessError as e:
            print("Command failed with error:\n", e.stderr)
            raise

def main(args=None):
    parser = argparse.ArgumentParser(description='Autogenerate Ground Truth Launcher')
    parser.add_argument('--pose_graph_filename', type=str, required=True, help='Path to the pose graph file')
    parser.add_argument('--output_filename', type=str, required=True, help='Path to the output relations file')
    parser.add_argument('--min_covered_distance', type=float, required=True, help='Minimum covered distance for ground truth generation')
    parser.add_argument('--outlier_threshold_meters', type=float, required=True, help='Outlier threshold in meters for ground truth generation')
    parser.add_argument('--outlier_threshold_radians', type=float, required=True, help='Outlier threshold in radians for ground truth generation')

    parsed_args = parser.parse_args()

    ground_truth_launcher = (
        CartographerAutogenerateGroundTruthLauncher()
        .set_pose_graph_filename(parsed_args.pose_graph_filename)
        .set_output_filename(parsed_args.output_filename)
        .set_min_covered_distance(parsed_args.min_covered_distance)
        .set_outlier_threshold_meters(parsed_args.outlier_threshold_meters)
        .set_outlier_threshold_radians(parsed_args.outlier_threshold_radians)
    )
    ground_truth_launcher.run()

if __name__ == '__main__':
    main()
