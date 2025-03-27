import itertools
import os
from pathlib import Path
from typing import Dict, List, Any, Tuple, Optional, Type
import tempfile
from dataclasses import dataclass
import time
import csv
import shutil
import numpy as np
import json

from ..config_manager import ConfigManager
from ..metric_calculators.base_calculator import BaseMetricCalculator
from ..metric_calculators.lua.pgm_based import LuaPgmMetricCalculator
from .base import BaseLauncher
from ..metric import Metric

class GridSearchOptimizer(BaseLauncher):
    """Grid search optimizer for Cartographer parameters.
    
    This optimizer evaluates different parameter combinations using PGM-based metrics
    to find the optimal configuration for Cartographer SLAM.
    
    Use topic remapping to specify the point cloud and IMU topics:
    --remap "/points2;/your_points_topic" --remap "/imu;/your_imu_topic"
    """
    
    # Use the unified metric calculator
    METRIC_CALCULATOR = LuaPgmMetricCalculator
    
    @classmethod
    def _register_params(cls):
        cls.register_parameter(
            name="bag_filename",
            param_type=str,
            required=True,
            help="Path to input bag file"
        )
        cls.register_parameter(
            name="config_dir",
            param_type=str,
            required=True,
            help="Directory containing Lua configs"
        )
        cls.register_parameter(
            name="config_basename",
            param_type=str,
            required=True,
            help="Base name of the Lua config file"
        )
        cls.register_parameter(
            name="parameter_grid",
            param_type=str,
            required=True,
            help="JSON string defining parameter grid."
        )
        cls.register_parameter(
            name="metrics",
            param_type=str,
            required=True,
            help="Comma-separated list of metrics to evaluate",
            default=None
        )
        cls.register_parameter(
            name="skip_seconds",
            param_type=int,
            required=False,
            help="Seconds to skip from bag start",
            default=0
        )
        cls.register_parameter(
            name="csv_output",
            param_type=str,
            required=False,
            help="Path to CSV file for storing grid search results",
            default=None
        )
    
    def __init__(self, **kwargs):
        """Initialize grid search optimizer."""
        super().__init__(**kwargs)
        
        # Parse parameter grid from JSON
        self.parameter_grid = json.loads(self._parameter_grid)

        # Parse metrics
        assert self._metrics is not None, f"Metrics must be specified. Available metrics: {', '.join(self.get_available_metrics())}"
        self.metrics = [m.strip() for m in self._metrics.split(',')]
        
        # Create base parameters dictionary
        self.base_params = {
            'bag_filename': self._bag_filename,
            'config_dir': self._config_dir,
            'config_basename': self._config_basename,
            'skip_seconds': self._skip_seconds
        }
        
        # Create temporary directory for configurations
        self.tmp_dir = Path(tempfile.mkdtemp(prefix="cartographer_grid_search_"))
        
        # Validate metrics
        self._validate_metrics(self.metrics)
    
    @classmethod
    def get_available_metrics(cls) -> List[str]:
        """Get all available metrics from the unified calculator."""
        return cls.METRIC_CALCULATOR.get_available_metrics()
    
    def _validate_metrics(self, metrics: List[str]) -> None:
        """Validate that requested metrics are available."""
        available_metrics = self.get_available_metrics()
        
        for metric in metrics:
            if metric not in available_metrics:
                raise ValueError(f"Unknown metric: {metric}. Available metrics: {', '.join(available_metrics)}")
        
    def _generate_parameter_combinations(self) -> List[Dict[str, Any]]:
        """Generate all possible parameter combinations from the grid."""
        param_names = list(self.parameter_grid.keys())
        param_values = list(self.parameter_grid.values())
        
        combinations = []
        for values in itertools.product(*param_values):
            param_dict = dict(zip(param_names, values))
            combinations.append(param_dict)
            
        return combinations
        
    def _evaluate_parameter_set(self, params: Dict[str, Any]) -> Dict[str, Dict[str, float]]:
        """Evaluate one parameter combination."""
        metrics_results = {}
        
        # Create temporary config with these parameters
        config_dir = self.tmp_dir / "configs"
        config_dir.mkdir(exist_ok=True)
        
        # Copy all config files from original config directory to temporary config directory
        original_config_dir = Path(self.base_params['config_dir'])
        for config_file in original_config_dir.glob("*.lua"):
            shutil.copy(config_file, config_dir)

        # Load and modify configuration
        config_manager = ConfigManager()
        config_manager.load_config(
            Path(self.base_params['config_dir']) / self.base_params['config_basename']
        )
        
        # Apply parameter modifications
        for param_path, value in params.items():
            config_manager[param_path] = value
        
        # Save modified config
        modified_config_path = config_dir / self.base_params['config_basename']
        with open(modified_config_path, "w") as f:
            f.write(config_manager.to_lua_string("options"))

        # Create a unique temporary directory for the calculator
        calculator_tmp_dir = self.tmp_dir / "pgm_metrics"
        
        # Prepare calculator arguments
        calculator_kwargs = dict(self.base_params)
        calculator_kwargs['config_dir'] = str(config_dir)
        calculator_kwargs['tmp_dir'] = str(calculator_tmp_dir)
        
        # Add remappings if any
        if self.remappings:
            calculator_kwargs['topic_remappings'] = [f"{src};{dst}" for src, dst in self.remappings]
        
        # Create calculator and run calculation
        print(f"Creating calculator with kwargs: {calculator_kwargs}")
        calculator = self.METRIC_CALCULATOR(**calculator_kwargs)
        print(f"Running calculator with metrics: {self.metrics}")
        results = calculator.calculate(self.metrics)
        
        # Store results
        for metric_name, metric in results.items():
            metrics_results[metric_name] = {
                'value': metric.value,
                'uncertainty': metric.uncertainty,
                'unit': metric.unit
            }
            
        return metrics_results
        
    def _write_csv_header(self, fieldnames: List[str]):
        """Write CSV header if output file is specified."""
        if not self._csv_output:
            return
            
        with open(self._csv_output, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

    def _write_csv_row(self, row_data: Dict[str, Any]):
        """Write a row to CSV if output file is specified."""
        if not self._csv_output:
            return
            
        with open(self._csv_output, 'a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=list(row_data.keys()))
            writer.writerow(row_data)

    def run(self) -> List[Metric]:
        """Run grid search to find best parameters for each metric."""
        combinations = self._generate_parameter_combinations()
        total_combinations = len(combinations)
        print(f"Evaluating {total_combinations} parameter combinations...")
        
        # Track best results for each metric
        best_results: Dict[str, Metric] = {}
        # Track best parameters for each metric
        best_parameters: Dict[str, Dict[str, Any]] = {}
        
        # Time tracking
        start_time = time.time()
        completed = 0
        
        # Initialize CSV if needed
        if self._csv_output:
            self.first_metrics = True
        
        for i, params in enumerate(combinations):
            iter_start = time.time()
            print(f"\nTesting combination {i+1}/{total_combinations}:")
            for param, value in params.items():
                print(f"  {param}: {value}")
                
            try:
                metrics = self._evaluate_parameter_set(params)
                
                # Write CSV header on first successful iteration
                if self._csv_output and self.first_metrics:
                    fieldnames = list(params.keys())  # Parameter names
                    for metric_name, data in metrics.items():
                        # Value and uncertainty columns for metrics
                        fieldnames.extend([f"{metric_name}_value", f"{metric_name}_uncertainty"])
                    self._write_csv_header(fieldnames)
                    self.first_metrics = False

                # Prepare and write CSV row
                if self._csv_output:
                    row_data = dict(params)  # Start with parameters
                    for metric_name, data in metrics.items():
                        # Value and uncertainty for metrics
                        row_data[f"{metric_name}_value"] = data['value']
                        row_data[f"{metric_name}_uncertainty"] = data['uncertainty']
                    self._write_csv_row(row_data)

                # Update best results for each metric
                for metric_name, data in metrics.items():
                    current_value = data['value']
                    current_uncertainty = data['uncertainty']
                    
                    if (metric_name not in best_results or 
                        current_value < best_results[metric_name].value):
                        best_results[metric_name] = Metric(
                            name=metric_name,
                            value=current_value,
                            uncertainty=current_uncertainty,
                            unit=data['unit']
                        )
                        best_parameters[metric_name] = params.copy()
                completed += 1
                
            except Exception as e:
                print(f"ERROR: Failed to evaluate parameter set:")
                for param, value in params.items():
                    print(f"    {param}: {value}")
                print(f"Error message: {str(e)}")
                print("Skipping this combination and continuing with next set...")
                continue
            
            # Time estimation
            iter_time = time.time() - iter_start
            elapsed_time = time.time() - start_time
            avg_time = elapsed_time / (i + 1)
            remaining = total_combinations - (i + 1)
            estimated_remaining = remaining * avg_time
            
            print(f"\nProgress: {i + 1}/{total_combinations}")
            print(f"Time for this iteration: {iter_time:.1f}s")
            print(f"Average time per iteration: {avg_time:.1f}s")
            print(f"Estimated remaining time: {estimated_remaining/60:.1f}m ({estimated_remaining/3600:.1f}h)")
                    
        if not best_results:
            raise RuntimeError("All parameter combinations failed! Check the errors above.")
        
        # Print final statistics
        total_time = time.time() - start_time
        print(f"\nGrid search completed:")
        print(f"Total time: {total_time/60:.1f}m ({total_time/3600:.1f}h)")
        print(f"Successful combinations: {completed}/{total_combinations}")
        print(f"Average time per iteration: {total_time/total_combinations:.1f}s")
        
        # Print results
        print("\nBest parameters for each metric:")
        for metric_name, metric in best_results.items():
            params = best_parameters[metric_name]
            print(f"\n{metric.name}:")
            print(f"  Best value: {metric.value:.5f} ± {metric.uncertainty:.5f} {metric.unit}")
            print("  Parameters:")
            for param, value in params.items():
                print(f"    {param}: {value}")
        
        # Return metrics with best parameters attached
        result_metrics = []
        for metric_name, metric in best_results.items():
            # Create a new metric object to avoid modifying the original
            result_metrics.append(metric)
        
        return result_metrics
    
    def generate_launch_description(self):
        """Not used for grid search, but required by BaseLauncher."""
        from launch import LaunchDescription
        return LaunchDescription([])


# Create main function for command-line usage
main = GridSearchOptimizer.generate_main()

if __name__ == "__main__":
    main()
