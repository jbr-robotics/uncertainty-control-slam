import itertools
import json
import shutil
import tempfile
from pathlib import Path
from typing import Dict, Any, Iterator, Tuple


class ConfigGridEnumerator:
    def __init__(self, grid_json: str, config_dir: str, config_basename: str):
        self._grid = self._parse_grid_json(grid_json)
        self._param_combinations = self._generate_combinations()
        self._config_dir = Path(config_dir)
        self._config_basename = config_basename

    def __iter__(self) -> Iterator[Tuple[Dict[str, Any], Path]]:
        for params in self._param_combinations:
            with tempfile.TemporaryDirectory() as tmp_dir:
                tmp_config_dir = Path(tmp_dir) / "configs"
                self._prepare_tmp_config_dir(tmp_config_dir)

                from cartographer_tuner.core.cartographer_config_manager import CartographerConfigManager
                config_manager = CartographerConfigManager()
                config_manager.load(self._config_dir / self._config_basename)

                for param_path, value in params.items():
                    config_manager[param_path] = value

                modified_config_path = tmp_config_dir / self._config_basename
                config_manager.save_to_file(modified_config_path)

                yield params, tmp_config_dir

    def _prepare_tmp_config_dir(self, config_dir: Path) -> None:
        config_dir.mkdir(parents=True, exist_ok=True)
        for config_file in self._config_dir.glob("*.lua"):
            shutil.copy(config_file, config_dir)

    def _parse_grid_json(self, data: str) -> Dict[str, Any]:
        try:
            return json.loads(data)
        except Exception:
            from cartographer_tuner.config_optimizers.exceptions import OptimizerInvalidArgumentException
            raise OptimizerInvalidArgumentException("Invalid JSON string for grid definition")

    def _generate_combinations(self) -> Iterator[Dict[str, Any]]:
        param_names, param_values = zip(*self._grid.items())
        return [dict(zip(param_names, values)) for values in itertools.product(*param_values)]
