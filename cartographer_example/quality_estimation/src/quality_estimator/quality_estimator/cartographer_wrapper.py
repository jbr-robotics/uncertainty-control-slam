from .ros2_wrapper import Ros2Wrapper


class CartographerOfflineNodeWrapper(Ros2Wrapper):
    def __init__(self):
        # Initialize default values for arguments
        self.collect_metrics = False
        self.configuration_directory = ""
        self.configuration_basenames = ""
        self.bag_filenames = ""
        self.urdf_filenames = ""
        self.use_bag_transforms = True
        self.load_state_filename = ""
        self.load_frozen_state = True
        self.save_state_filename = ""
        self.keep_running = False
        self.skip_seconds = 0.0

    def set_collect_metrics(self, value: bool):
        self.collect_metrics = value
        return self

    def set_configuration_directory(self, value: str):
        self.configuration_directory = value
        return self

    def set_configuration_basenames(self, value: str):
        self.configuration_basenames = value
        return self

    def set_bag_filenames(self, value: str):
        self.bag_filenames = value
        return self

    def set_urdf_filenames(self, value: str):
        self.urdf_filenames = value
        return self

    def set_use_bag_transforms(self, value: bool):
        self.use_bag_transforms = value
        return self

    def set_load_state_filename(self, value: str):
        self.load_state_filename = value
        return self

    def set_load_frozen_state(self, value: bool):
        self.load_frozen_state = value
        return self

    def set_save_state_filename(self, value: str):
        self.save_state_filename = value
        return self

    def set_keep_running(self, value: bool):
        self.keep_running = value
        return self

    def set_skip_seconds(self, value: float):
        self.skip_seconds = value
        return self

    def build_command(self):
        # Construct the command for running the node
        command = [
            "ros2", "run", "cartographer_ros", "cartographer_offline_node",
            f"--collect_metrics={'true' if self.collect_metrics else 'false'}",
            f"--configuration_directory={self.configuration_directory}",
            f"--configuration_basenames={self.configuration_basenames}",
            f"--bag_filenames={self.bag_filenames}",
            f"--urdf_filenames={self.urdf_filenames}",
            f"--use_bag_transforms={'true' if self.use_bag_transforms else 'false'}",
            f"--load_state_filename={self.load_state_filename}",
            f"--load_frozen_state={'true' if self.load_frozen_state else 'false'}",
            f"--save_state_filename={self.save_state_filename}",
            f"--keep_running={'true' if self.keep_running else 'false'}",
            f"--skip_seconds={self.skip_seconds}"
        ]
        return command
