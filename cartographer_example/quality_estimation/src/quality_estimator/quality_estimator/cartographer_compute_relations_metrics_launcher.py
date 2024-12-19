import argparse
import subprocess

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

    def run(self):
        if not self._relations_filename or not self._pose_graph_filename:
            raise ValueError("Both 'relations_filename' and 'pose_graph_filename' must be set before running the application.")

        command = [
            "cartographer_compute_relations_metrics",
            "-relations_filename", self._relations_filename,
            "-pose_graph_filename", self._pose_graph_filename
        ]

        # Forward stdout and stderr to the parent process
        subprocess.run(command, check=True, stdout=None, stderr=None)

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