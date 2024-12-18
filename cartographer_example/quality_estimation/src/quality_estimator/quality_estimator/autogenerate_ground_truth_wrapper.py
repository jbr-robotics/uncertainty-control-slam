from .ros2_wrapper import Ros2Wrapper


class CartographerAutogenerateGroundTruthWrapper(Ros2Wrapper):
    def __init__(self):
        # Initialize default values for arguments
        self.pose_graph_filename = ""
        self.output_filename = ""
        self.min_covered_distance = 100.0
        self.outlier_threshold_meters = 0.15
        self.outlier_threshold_radians = 0.02

    def set_pose_graph_filename(self, value: str):
        self.pose_graph_filename = value
        return self

    def set_output_filename(self, value: str):
        self.output_filename = value
        return self

    def set_min_covered_distance(self, value: float):
        self.min_covered_distance = value
        return self

    def set_outlier_threshold_meters(self, value: float):
        self.outlier_threshold_meters = value
        return self

    def set_outlier_threshold_radians(self, value: float):
        self.outlier_threshold_radians = value
        return self

    def build_command(self):
        # Construct the command for running the autogenerate ground truth tool
        command = [
            "ros2", "run", "cartographer_ros", "cartographer_autogenerate_ground_truth",
            f"--pose_graph_filename={self.pose_graph_filename}",
            f"--output_filename={self.output_filename}",
            f"--min_covered_distance={self.min_covered_distance}",
            f"--outlier_threshold_meters={self.outlier_threshold_meters}",
            f"--outlier_threshold_radians={self.outlier_threshold_radians}"
        ]
        return command
