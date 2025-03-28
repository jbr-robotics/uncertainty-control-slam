import tempfile
from pathlib import Path
from typing import Dict, List, Optional

from cartographer_tuner.metrics.calculators.base_metric_calculator import BaseMetricCalculator
from cartographer_tuner.metrics.metric import Metric
from cartographer_tuner.metrics.calculators.pgm.corner_count_calculator import CornerCountCalculator
from cartographer_tuner.metrics.calculators.pgm.enclosed_areas_calculator import EnclosedAreasCalculator
from cartographer_tuner.metrics.calculators.pgm.occupied_proportion_calculator import OccupiedProportionCalculator
from cartographer_tuner.metrics.calculators.lua.base_lua_calculator import BaseLuaMetricCalculator
from cartographer_tuner.tools.combinations.lua_to_pgm import LuaToPgmLauncher

__all__ = ["LuaPgmMetricCalculator"]

class LuaPgmMetricCalculator(BaseLuaMetricCalculator):
    """Unified calculator for all PGM-based metrics using Lua configurations.
    """

    CALCULATORS = [
        CornerCountCalculator,
        EnclosedAreasCalculator,
        OccupiedProportionCalculator
    ]

    def __init__(
        self,
        bag_filename: str,
        config_dir: str,
        config_basename: str,
        skip_seconds: int = 0,
        resolution: float = 0.05,
        **kwargs
    ):
        super().__init__(
            bag_filename=bag_filename,
            config_dir=config_dir,
            config_basename=config_basename,
            skip_seconds=skip_seconds,
        )
        
        self.resolution = resolution
    
        self.lua_to_pgm_launcher = None
        self.calculators = {
            calculator: None for calculator in LuaPgmMetricCalculator.CALCULATORS
        }
        self.kwargs = kwargs

    @classmethod
    def available_metrics(cls) -> List[str]:
        res = []
        for calc in cls.CALCULATORS:
            res.extend(calc.available_metrics())
        return res

    def _map_filestem(self, directory: Path) -> str:
        return str(directory / Path(self.bag_filename).stem)
    
    def generate_map(self, directory: Path) -> None:
        self.lua_to_pgm_launcher = LuaToPgmLauncher(
            bag_filename=str(self.bag_filename),
            configuration_directory=str(self.config_dir),
            configuration_basename=str(self.config_basename),
            skip_seconds=self.skip_seconds,
            map_filestem=self._map_filestem(directory),
            resolution=self.resolution,
        )
        
        self.lua_to_pgm_launcher.run()
        
    def _group_metrics(self, metrics: List[str]) -> Dict[str, List[str]]:
        return {
            calculator: [m for m in metrics if m in calculator.available_metrics()]
            for calculator in LuaPgmMetricCalculator.CALCULATORS
        }
    
    def calculate(self, metrics: Optional[List[str]] = None) -> Dict[str, Metric]:
        metrics = self._process_metric_names(metrics)
        grouped_metrics = self._group_metrics(metrics)
        
        results = {}

        with tempfile.TemporaryDirectory() as tmp_dir:
            self.generate_map(Path(tmp_dir))
            pgm_path = str(self.lua_to_pgm_launcher.pgm_path)
            yaml_path = str(self.lua_to_pgm_launcher.yaml_path)
        
            for calculator, metrics in grouped_metrics.items():
                calc = calculator(
                    map_path=pgm_path,
                    yaml_path=yaml_path, 
                    **self.kwargs
                )
                res = calc.calculate(metrics)
                results.update(res)
        
        return results
