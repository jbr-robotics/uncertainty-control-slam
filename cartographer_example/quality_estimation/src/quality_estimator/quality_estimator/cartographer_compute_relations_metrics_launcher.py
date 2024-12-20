import argparse
import subprocess
import sys

class CartographerComputeRelationsMetricsLauncher:
    def __init__(self):
        self._relations_filename = None
        self._pose_graph_filename = None

    def set_relations_filename(self, filename: str) -> "CartographerComputeRelationsMetricsLauncher":
        self._relations_filename = filename
        return self

    def set_pose_graph_filename(self, filename: str) -> "CartographerComputeRelationsMetricsLauncher":
        self._pose_graph_filename = filename
        return self

    @staticmethod
    def parse_metrics(output: str) -> dict:
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

    def run(self):
        if not self._relations_filename or not self._pose_graph_filename:
            raise ValueError("Both 'relations_filename' and 'pose_graph_filename' must be set before running the application.")

        command = [
            "cartographer_compute_relations_metrics",
            "-relations_filename", self._relations_filename,
            "-pose_graph_filename", self._pose_graph_filename
        ]

        # Run command and capture output while still forwarding to stdout/stderr
        result = subprocess.run(command, check=True, capture_output=True, text=True)
        
        # Print original output
        output = result.stdout 
        print(result.stdout)
        if result.stderr:
            print(result.stderr, file=sys.stderr)
            output = result.stderr

        return self.parse_metrics(output)

def main():
    parser = argparse.ArgumentParser(description="Launcher for cartographer_compute_relations_metrics")
    parser.add_argument(
        "--relations_filename",
        type=str,
        required=True,
        help="Path to the relations file (e.g., /path/to/relations.pbstream)"
    )
    parser.add_argument(
        "--pose_graph_filename",
        type=str,
        required=True,
        help="Path to the pose graph file (e.g., /path/to/pose_graph.pbstream)"
    )

    args = parser.parse_args()

    launcher = (
        CartographerComputeRelationsMetricsLauncher()
        .set_relations_filename(args.relations_filename)
        .set_pose_graph_filename(args.pose_graph_filename)
    )

    try:
        launcher.run()
    except subprocess.CalledProcessError as e:
        print(f"Error: Command failed with exit code {e.returncode}.")
        raise

if __name__ == "__main__":
    main()