from typing import Optional, Dict
import subprocess
import sys
from quality_estimator.launchers.base import BaseLauncher
from launch import LaunchDescription

class CartographerComputeRelationsMetricsLauncher(BaseLauncher):
    # TODO: this class is not tested
    """Launch Cartographer's relations metrics computation tool."""

    # Register all parameters
    BaseLauncher.register_parameter(
        "relations_filename",
        str,
        required=True,
        help="Path to the relations file (e.g., /path/to/relations.pbstream)"
    )
    BaseLauncher.register_parameter(
        "pose_graph_filename",
        str,
        required=True,
        help="Path to the pose graph file (e.g., /path/to/pose_graph.pbstream)"
    )

    def generate_launch_description(self) -> LaunchDescription:
        """This launcher doesn't use ROS2 launch system, so this method is not used."""
        return LaunchDescription([])

    @staticmethod
    def parse_metrics(output: str) -> Dict:
        """Parse metrics from cartographer_compute_relations_metrics output.
        
        Args:
            output: String output from the metrics computation
            
        Returns:
            Dictionary containing parsed metrics with values and uncertainties
        """
        metrics = {}
        for line in output.split('\n'):
            # Look for lines containing error metrics
            if any(x in line for x in ['translational error', 'rotational error']):
                # Split line into parts
                parts = line.strip().split()
                # Extract metric name (e.g., "Abs translational error")
                name = ' '.join(parts[:-4])  # Everything before the numbers
                # Extract value and uncertainty
                value = float(parts[-4])
                uncertainty = float(parts[-2])
                unit = parts[-1]
                metrics[name] = {
                    'value': value,
                    'uncertainty': uncertainty,
                    'unit': unit
                }
        return metrics

    def run(self) -> dict:
        """Execute the relations metrics computation command.
        
        This overrides the base class run() method to execute a direct command
        instead of using the ROS2 launch system.
        
        Returns:
            Dictionary containing the parsed metrics
        """
        command = [
            "cartographer_compute_relations_metrics",
            "-relations_filename", self._relations_filename,
            "-pose_graph_filename", self._pose_graph_filename
        ]

        try:
            # Run command and capture output while still forwarding to stdout/stderr
            result = subprocess.run(command, check=True, capture_output=True, text=True)
            
            # Print original output
            output = result.stdout
            print(output)
            if result.stderr:
                print(result.stderr, file=sys.stderr)
                output = result.stderr

            # Parse and return metrics
            return self.parse_metrics(output)
        except subprocess.CalledProcessError as e:
            print("cartographer_compute_relations_metrics failed with error:", e)
            raise

# Generate main function
main = CartographerComputeRelationsMetricsLauncher.generate_main()

if __name__ == '__main__':
    main()
