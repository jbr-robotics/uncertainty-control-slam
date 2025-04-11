from pathlib import Path
from typing import Dict, Any
import json

from cartographer_tuner.config_optimizers.config_grid_enumerator import ConfigGridEnumerator
from cartographer_tuner.tools.combinations.submap_sampler_cartographer import SubmapSamplerCartographer
from cartographer_tuner.utils.terminal_runnable import TerminalRunnable


class SubmapGridSearch(TerminalRunnable):
    """Runs submap sampling over an arbitrary grid of configuration parameters."""

    @classmethod
    def _register_params(cls):
        cls.register_parameter("grid", str, True, "JSON string or path defining the parameter grid")
        cls.register_parameter("config_dir", str, True, "Directory with config files")
        cls.register_parameter("config_basename", str, True, "Base name of the Lua config file")
        cls.register_parameter("bag_filename", str, True, "Path to input bag file")
        cls.register_parameter("output_root", Path, True, "Output directory for submap samples")
        cls.register_parameter("samples", int, True, "Number of submaps to generate", default=1)
        cls.register_parameter("rate", float, False, "Playback rate for the bag file", default=1.0)
        cls.register_parameter("skip_begin", float, False, "Seconds to skip at the start of the bag file", default=0.0)
        cls.register_parameter("skip_end", float, False, "Seconds to skip at the end of the bag file", default=0.0)

    def run(self):
        try:
            grid_dict = json.loads(self._grid)
        except json.JSONDecodeError as e:
            raise ValueError(f"Failed to parse grid_json. Make sure it’s a valid JSON string.\nError: {e}")

        enumerator = ConfigGridEnumerator(
            grid_json=json.dumps(grid_dict),
            config_dir=self._config_dir,
            config_basename=self._config_basename
        )

        for param_set, tmp_config_path in enumerator:
            output_dir_name = self._format_output_dir_name(param_set)
            output_path = self._output_root / output_dir_name
            output_path.mkdir(parents=True, exist_ok=True)

            print(f"\n==> Running for config: {param_set}, output path: {output_path}")

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

    def _format_output_dir_name(self, param_set: Dict[str, Any]) -> str:
        value_str = "_".join(str(v) for v in param_set.values())
        return f"{value_str}"
