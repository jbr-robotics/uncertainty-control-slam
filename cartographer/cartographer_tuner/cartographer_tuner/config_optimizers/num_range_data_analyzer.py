from pathlib import Path
from typing import List
import json

from cartographer_tuner.config_optimizers.config_grid_enumerator import ConfigGridEnumerator
from cartographer_tuner.tools.combinations.submap_sampler_cartographer import SubmapSamplerCartographer
from cartographer_tuner.utils.terminal_runnable import TerminalRunnable


class NumRangeDataAnalyzer(TerminalRunnable):
    """Runs submap sampling for different values of num_accumulated_range_data."""

    @classmethod
    def _register_params(cls):
        cls.register_parameter(
            "num_range_data",
            param_type=int,
            required=True,
            help="List of values for trajectory_builder.trajectory_builder_2d.num_accumulated_range_data",
            nargs="+"
        )
        cls.register_parameter("config_dir", str, True, "Directory with config files")
        cls.register_parameter("config_basename", str, True, "Base name of the Lua config file")
        cls.register_parameter("bag_filename", str, True, "Path to input bag file")
        cls.register_parameter("output_root", Path, True, "Output directory for submap samples")
        cls.register_parameter("samples", int, True, "Number of submaps to generate", default=1)
        cls.register_parameter("rate", float, False, "Playback rate for the bag file", default=1.0)
        cls.register_parameter("skip_begin", float, False, "Number of seconds to be skipped at the beginning of bag file", default=0.0)
        cls.register_parameter("skip_end", float, False, "Number of seconds to be skipped at the end of bag file", default=0.0)

    def run(self):
        grid_json = self._build_grid_json(self._num_range_data)

        enumerator = ConfigGridEnumerator(
            grid_json=grid_json,
            config_dir=self._config_dir,
            config_basename=self._config_basename
        )

        for param_set, tmp_config_path in enumerator:
            value = param_set[self.num_range_data_name()]
            output_path = self._output_root / str(value)
            output_path.mkdir(parents=True, exist_ok=True)

            print(f"\n==> Running for num_accumulated_range_data = {value}")

            sampler = SubmapSamplerCartographer(
                bag_filename=self._bag_filename,
                configuration_directory=str(tmp_config_path),
                configuration_basename=self._config_basename,
                samples=self._samples,
                output_path=output_path,
                rate=self._rate,
                skip_begin=self._skip_begin,
                skip_end=self._skip_end,
            )
            sampler.run()

    def _build_grid_json(self, values: List[int]) -> str:
        return json.dumps({
            self.num_range_data_name(): values
        })

    @staticmethod
    def num_range_data_name() -> str:
        return "trajectory_builder.trajectory_builder_2d.num_accumulated_range_data"