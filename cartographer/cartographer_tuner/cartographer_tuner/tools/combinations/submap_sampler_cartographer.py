from pathlib import Path
import threading
import time
import rclpy
import subprocess
from rclpy.node import Node
from cartographer_ros_msgs.msg import SubmapList
import rosbag2_py
from multiprocessing import Process

from cartographer_tuner.utils.terminal_runnable import TerminalRunnable
from cartographer_tuner.tools.combinations.first_submap_cartographer import FirstSubmapCartographerLauncher


class SubmapSamplerCartographer(TerminalRunnable):
    @classmethod
    def _register_params(cls):
        cls.register_parameter(
            "bag_filename",
            str,
            required=True,
            help="Path to the bag file"
        )
        cls.register_parameter(
            "configuration_directory",
            str,
            required=True,
            help="Directory containing Lua configuration files"
        )
        cls.register_parameter(
            "configuration_basename",
            str,
            required=True,
            help="Base name of the Lua configuration file"
        )
        cls.register_parameter(
            "rate",
            float,
            required=False,
            default=1.0,
            help="Rate at which to play back the bag file"
        )
        cls.register_parameter(
            "samples",
            int,
            required=True,
            help="Number of submap samples"
        )
        cls.register_parameter(
            "output_path",
            Path,
            required=True,
            help="Path to directory where to save samples"
        )
        cls.register_parameter("skip_begin", float, False, "Number of seconds to be skipped at the beginning of bag file", default=0.0)
        cls.register_parameter("skip_end", float, False, "Number of seconds to be skipped at the end of bag file", default=0.0)

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        
        Path(self._output_path).mkdir(parents=True, exist_ok=True)
        self._bag_duration = self.get_bag_duration(self._bag_filename).nanoseconds / 1e9
        assert self._bag_duration > self._skip_begin + self._skip_end

    def _run_single_launcher(self, sample: int):
        output_path = self._output_path / f"submap_{sample}.pkl"
        start_offset = self._skip_begin + (sample / self._samples) * (self._bag_duration - self._skip_begin - self._skip_end)

        launcher = FirstSubmapCartographerLauncher(
            bag_filename=self._bag_filename,
            configuration_directory=self._configuration_directory,
            configuration_basename=self._configuration_basename,
            output_path=output_path,
            rate=self._rate,
            start_offset=start_offset
        )
        launcher.run()

    def run(self):
        
        for sample in range(self._samples):
            p = Process(target=self._run_single_launcher, args=(sample,))
            p.start()
            p.join()

    @staticmethod
    def get_bag_duration(bag_path):
        metadata = rosbag2_py.Info().read_metadata(bag_path, 'sqlite3')
        duration = metadata.duration
        return duration