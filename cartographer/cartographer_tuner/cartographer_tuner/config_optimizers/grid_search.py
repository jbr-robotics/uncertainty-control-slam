from cartographer_tuner.config_optimizers.base_config_optimizer import BaseOptimizer
from cartographer_tuner.metrics.calculators.lua.pgm_based_calculator import LuaPgmMetricCalculator
from cartographer_tuner.metrics.metric import Metric
from cartographer_tuner.utils.csv_stats_writer import CsvStatsWriter
from cartographer_tuner.config_optimizers.config_grid_enumerator import ConfigGridEnumerator

import time
from typing import Dict, List


class GridSearchConfigOptimizer(BaseOptimizer):
    CALCULATORS = [LuaPgmMetricCalculator]

    @classmethod
    def _register_params(cls):
        super()._register_params()
        cls.register_parameter("output", str, False, "Path to CSV file", default=None)
        cls.register_parameter("grid", str, True, "JSON string defining parameter grid")

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._metrics = self._preprocess_metric_names(self._parse_metric_names(self._metrics))
        self._writer = CsvStatsWriter(self._output)

    def run(self) -> Dict[str, Metric]:
        enumerator = ConfigGridEnumerator(self._grid, self._config_dir, self._config_basename)
        total = len(enumerator._param_combinations)
        print(f"Evaluating {total} parameter combinations...")

        best_results, best_parameters = {}, {}
        start_time = time.time()

        for i, (params, config_dir) in enumerate(enumerator):
            print(f"\nTesting combination {i+1}/{total}: {params}")
            iter_start = time.time()

            try:
                metrics = self._evaluate_metrics(params, config_dir)
            except Exception as e:
                print(f"ERROR during evaluation: {e}")
                continue

            self._writer.write(params, metrics)

            for key, metric in metrics.items():
                if key not in best_results or metric.value < best_results[key].value:
                    best_results[key] = metric
                    best_parameters[key] = params

            self._log_progress(i, total, iter_start, start_time)

        if not best_results:
            raise RuntimeError("All parameter combinations failed!")

        print("Best results:")
        for key, metric in best_results.items():
            print(f"{metric}\nat parameters: {best_parameters[key]}")

        return best_results

    def _evaluate_metrics(self, params, config_dir) -> Dict[str, Metric]:
        results = {}
        for calculator in self.CALCULATORS:
            calc = calculator(
                bag_filename=self._bag_filename,
                config_dir=str(config_dir),
                config_basename=self._config_basename
            )
            results.update(calc.calculate(self._metrics))
        return results

    def _log_progress(self, index, total, iter_start, start_time):
        elapsed = time.time() - start_time
        avg = elapsed / (index + 1)
        remaining = (total - index - 1) * avg
        print(f"Progress: {index + 1}/{total}")
        print(f"Iteration time: {time.time() - iter_start:.1f}s | Avg: {avg:.1f}s | Remaining: {remaining/60:.1f}m")
    
    @classmethod
    def available_metrics(cls) -> List[str]:
        return [metric for calc in cls.CALCULATORS for metric in calc.available_metrics()]

    @staticmethod
    def _parse_metric_names(metric_names: str) -> List[str]:
        return [x.strip() for x in metric_names.split(",")]

    @classmethod
    def _preprocess_metric_names(cls, metric_names: List[str]) -> List[str]:
        available = cls.available_metrics()
        if not any(name in available for name in metric_names):
            raise ValueError(f"Invalid metric. Available: {available}")
        return metric_names
