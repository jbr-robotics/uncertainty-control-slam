
import itertools
import json
from typing import List, Optional, Dict, Any
from pathlib import Path
import shutil
import time
import tempfile

from cartographer_tuner.config_optimizers.base_config_optimizer import BaseOptimizer
from cartographer_tuner.metrics.calculators.lua.pgm_based_calculator import LuaPgmMetricCalculator
from cartographer_tuner.config_optimizers.exceptions import OptimizerInvalidArgumentException
from cartographer_tuner.utils.csv_stats_writer import CsvStatsWriter
from cartographer_tuner.core.cartographer_config_manager import CartographerConfigManager
from cartographer_tuner.metrics.metric import Metric

class GridSearchConfigOptimizer(BaseOptimizer):

    CALCULATORS = [
        LuaPgmMetricCalculator
    ]

    @classmethod
    def _register_params(cls):
        super()._register_params()
        cls.register_parameter(
            name="output",
            param_type=str,
            required=False,
            help="Path to CSV file for storing grid search results",
            default=None
        )
        cls.register_parameter(
            name="grid",
            param_type=str,
            required=True,
            help="JSON string defining parameter grid."
        )

    def run(self) -> None:
        pass

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self._grid = self._parse_grid_json(self._grid)
        metrics = self._parse_metric_names(self._metrics)
        self._metrics = self._preprocess_metric_names(metrics)
        self._writer = CsvStatsWriter(self._output)

    def _evaluate_parameter_set(self, params: Dict[str, Any], directory: Path) -> Dict[str, Dict[str, float]]:
        results = {}

        config_dir = directory / "configs"

        self.prepare_tmp_config_dir(config_dir)
        
        config_manager = CartographerConfigManager()
        config_manager.load(Path(self._config_dir) / self._config_basename)
        
        for param_path, value in params.items():
            config_manager[param_path] = value
        
        modified_config_path = config_dir / self._config_basename
        config_manager.save_to_file(modified_config_path)

        for calculator in self.CALCULATORS:
            calc = calculator(
                bag_filename=self._bag_filename,
                config_dir=str(config_dir),
                config_basename=self._config_basename
            )
            res = calc.calculate(self._metrics)
            results.update(res)
        return results
    
    def run(self) -> Dict[str, Metric]:
        """Run grid search to find best parameters for each metric."""

        combinations = self._generate_parameter_combinations()
        total_combinations = len(combinations)
        print(f"Evaluating {total_combinations} parameter combinations...")
        
        best_results: Dict[str, Metric] = {}
        best_parameters: Dict[str, Dict[str, Any]] = {}
        
        start_time = time.time()
        
        for i, params in enumerate(combinations):
            iter_start = time.time()
            print(f"\nTesting combination {i+1}/{total_combinations}: {params}")
                
            try:
                with tempfile.TemporaryDirectory() as tmp_dir:
                    metrics = self._evaluate_parameter_set(params, Path(tmp_dir))
            except Exception as e:
                print(f"ERROR: Failed to evaluate parameter set:")
                for param, value in params.items():
                    print(f"    {param}: {value}")
                print(f"Error message: {str(e)}")
                print("Skipping this combination and continuing with next set...")
                continue

            self._writer.write(params, metrics)

            for key, metric in metrics.items():
                if key not in best_results or metric.value < best_results[key].value:
                    best_results[key] = metric
                    best_parameters[key] = params.copy()
            
            # Time estimation
            iter_time = time.time() - iter_start
            elapsed_time = time.time() - start_time
            avg_time = elapsed_time / (i + 1)
            remaining = total_combinations - (i + 1)
            estimated_remaining = remaining * avg_time
            
            # Logging
            print(f"\nProgress: {i + 1}/{total_combinations}")
            print(f"Time for this iteration: {iter_time:.1f}s")
            print(f"Average time per iteration: {avg_time:.1f}s")
            print(f"Estimated remaining time: {estimated_remaining/60:.1f}m ({estimated_remaining/3600:.1f}h)")
                    
        if not best_results:
            raise RuntimeError("All parameter combinations failed! Check the errors above.")
        
        # Logging
        print(f"Best results:")
        for key, metric in best_results.items():
            print(f"=================\n"
                  f"{metric}\n"
                  f"at parameters: {best_parameters[key]}"
                )

        return best_results


    @classmethod
    def available_metrics(cls) -> List[str]:
        res = []
        for calculator in cls.CALCULATORS:
            res.extend(calculator.available_metrics())
        return res
    
    @staticmethod
    def _parse_grid_json(data: str) -> Dict[str, Any]:
        try:
            return json.loads(data)
        except Exception as e:
            raise OptimizerInvalidArgumentException("Invalid JSON string for grid definition")
        
    @staticmethod
    def _parse_metric_names(metric_names: str) -> List[str]:
        res = [x.strip() for x in metric_names.split(",")]
        return res
    
    @classmethod
    def _preprocess_metric_names(cls, metric_names: Optional[List[str]] = None) -> List[str]:
        available_metrics = cls.available_metrics()
        if metric_names is None:
            return available_metrics
        for metric_name in metric_names:
            if metric_name in available_metrics:
                break
        else:  
            raise OptimizerInvalidArgumentException(f"Invalid metric name: {metric_name}, expected one of: {available_metrics}")
        return metric_names
        
        
    def _generate_parameter_combinations(self) -> List[Dict[str, Any]]:
        param_names, param_values = zip(*self._grid.items())
        
        combinations = []
        for values in itertools.product(*param_values):
            param_dict = dict(zip(param_names, values))
            combinations.append(param_dict)
            
        return combinations
    
    def prepare_tmp_config_dir(self, config_dir: Path) -> None:
        config_dir.mkdir(exist_ok=True)
        
        original_config_dir = Path(self._config_dir)
        for config_file in original_config_dir.glob("*.lua"):
            shutil.copy(config_file, config_dir)

