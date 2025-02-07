import argparse
import itertools
import os
from pathlib import Path
from typing import Dict, List, Any
import tempfile
from dataclasses import dataclass
import time
import csv
import shutil

from .quality_estimator import QualityEstimator
from .map_metric_estimator import MapMetricEstimator
from .config_manager import ConfigManager

@dataclass
class ParameterSearchResult:
    """Container for parameter search results."""
    metric_name: str
    best_value: float
    best_uncertainty: float
    best_parameters: Dict[str, Any]
    unit: str

class ParameterOptimizer:
    def __init__(self,
                 bag_filename: str,
                 config_dir: str,
                 config_basename: str,
                 points2_topic: str,
                 imu_topic: str,
                 parameter_grid: Dict[str, List[Any]],
                 metrics: List[str],
                 min_covered_distance: float = 20.0,
                 outlier_threshold_meters: float = 0.15,
                 outlier_threshold_radians: float = 0.02,
                 skip_seconds: int = 0,
                 csv_output: str = None):
        """Initialize parameter optimizer.
        
        Args:
            bag_filename: Path to input bag file
            config_dir: Directory containing Lua configs
            config_basename: Base name of config file
            points2_topic: Topic name for point cloud data
            imu_topic: Topic name for IMU data
            parameter_grid: Dictionary mapping parameter names to lists of values
            metrics: List of metrics to evaluate ('relational', 'mme', 'mpv', 'mom')
            min_covered_distance: Minimum distance for ground truth generation
            outlier_threshold_meters: Distance threshold for outliers
            outlier_threshold_radians: Angular threshold for outliers
            skip_seconds: Seconds to skip from bag start
            csv_output: Optional path to CSV file for storing results
        """
        self.base_params = {
            'bag_filename': bag_filename,
            'config_dir': config_dir,
            'config_basename': config_basename,
            'points2_topic': points2_topic,
            'imu_topic': imu_topic,
            'min_covered_distance': min_covered_distance,
            'outlier_threshold_meters': outlier_threshold_meters,
            'outlier_threshold_radians': outlier_threshold_radians,
            'skip_seconds': skip_seconds
        }
        self.parameter_grid = parameter_grid
        self.metrics = metrics
        self.csv_output = csv_output
        
        # Create temporary directory for configurations
        self.tmp_dir = Path(tempfile.mkdtemp(prefix="cartographer_grid_search_"))
        
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
        metrics = {}
        
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

        # Get relational metrics if requested
        if 'relational' in self.metrics:
            estimator_params = dict(self.base_params)
            estimator_params['config_dir'] = str(config_dir)
            estimator_params['tmp_dir'] = str(self.tmp_dir / "estimator")
            
            estimator = QualityEstimator(**estimator_params)
            metrics.update(estimator.run())

        # Get map metrics if any requested
        map_metrics = set(self.metrics) & {'mme', 'mpv', 'mom'}
        if map_metrics:
            map_estimator = MapMetricEstimator(
                bag_filename=self.base_params['bag_filename'],
                config_dir=str(config_dir),
                config_basename=self.base_params['config_basename'],
                points2_topic=self.base_params['points2_topic'],
                imu_topic=self.base_params['imu_topic'],
                metrics=list(map_metrics),
                skip_seconds=self.base_params['skip_seconds'],
                tmp_dir=str(self.tmp_dir / "map_metrics")
            )
            
            # Convert map metrics to match interface with dummy uncertainty and units
            map_results = map_estimator.run()
            for metric, value in map_results.items():
                metrics[metric] = {
                    'value': value,
                    'uncertainty': 0.0,
                    'unit': 'nats' if metric == 'mme' else '?' if metric == 'mpv' else '?'
                }
            
        return metrics
        
    def _write_csv_header(self, fieldnames: List[str]):
        """Write CSV header if output file is specified."""
        if not self.csv_output:
            return
            
        with open(self.csv_output, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

    def _write_csv_row(self, row_data: Dict[str, Any]):
        """Write a row to CSV if output file is specified."""
        if not self.csv_output:
            return
            
        with open(self.csv_output, 'a', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=list(row_data.keys()))
            writer.writerow(row_data)

    def find_best_parameters(self) -> List[ParameterSearchResult]:
        """Find best parameters for each metric."""
        combinations = self._generate_parameter_combinations()
        total_combinations = len(combinations)
        print(f"Evaluating {total_combinations} parameter combinations...")
        
        # Track best results for each metric
        best_results: Dict[str, ParameterSearchResult] = {}
        
        # Time tracking
        start_time = time.time()
        completed = 0
        
        # Initialize CSV if needed
        if self.csv_output:
            self.first_metrics = True
        
        for i, params in enumerate(combinations):
            iter_start = time.time()
            print(f"\nTesting combination {i+1}/{total_combinations}:")
            for param, value in params.items():
                print(f"  {param}: {value}")
                
            try:
                metrics = self._evaluate_parameter_set(params)
                
                # Write CSV header on first successful iteration
                if self.csv_output and self.first_metrics:
                    fieldnames = list(params.keys())  # Parameter names
                    for metric_name, data in metrics.items():
                        if metric_name in {'mme', 'mpv', 'mom'}:
                            # Single column for map metrics
                            fieldnames.append(metric_name)
                        else:
                            # Value and uncertainty columns for relational metrics
                            fieldnames.extend([f"{metric_name}_value", f"{metric_name}_uncertainty"])
                    self._write_csv_header(fieldnames)
                    self.first_metrics = False

                # Prepare and write CSV row
                if self.csv_output:
                    row_data = dict(params)  # Start with parameters
                    for metric_name, data in metrics.items():
                        if metric_name in {'mme', 'mpv', 'mom'}:
                            # Single value for map metrics
                            row_data[metric_name] = data['value']
                        else:
                            # Value and uncertainty for relational metrics
                            row_data[f"{metric_name}_value"] = data['value']
                            row_data[f"{metric_name}_uncertainty"] = data['uncertainty']
                    self._write_csv_row(row_data)

                # Update best results for each metric
                for metric_name, data in metrics.items():
                    current_value = data['value']
                    current_uncertainty = data['uncertainty']
                    
                    if (metric_name not in best_results or 
                        current_value < best_results[metric_name].best_value):
                        best_results[metric_name] = ParameterSearchResult(
                            metric_name=metric_name,
                            best_value=current_value,
                            best_uncertainty=current_uncertainty,
                            best_parameters=params.copy(),
                            unit=data['unit']
                        )
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
        
        return list(best_results.values())

def main():
    parser = argparse.ArgumentParser(description="Optimize Cartographer parameters via grid search")
    
    # Base parameters
    parser.add_argument("--bag_filename", required=True,
                      help="Path to input bag file")
    parser.add_argument("--config_dir", required=True,
                      help="Directory containing Lua configs")
    parser.add_argument("--config_basename", required=True,
                      help="Base name of config file")
    parser.add_argument("--points2_topic", required=True,
                      help="Point cloud topic name")
    parser.add_argument("--imu_topic", required=True,
                      help="IMU topic name")
    
    # Grid search parameters
    parser.add_argument("--parameter_grid", required=True,
                      help="JSON string defining parameter grid, e.g., "
                      '\'{"TRAJECTORY_BUILDER.max_range": [15, 20], '
                      '"POSE_GRAPH.optimize_every_n_nodes": [30, 60]}\'')
    
    # Optional parameters
    parser.add_argument("--min_covered_distance", type=float, default=20.0,
                      help="Minimum distance for ground truth generation")
    parser.add_argument("--outlier_threshold_meters", type=float, default=0.15,
                      help="Distance threshold for outliers")
    parser.add_argument("--outlier_threshold_radians", type=float, default=0.02,
                      help="Angular threshold for outliers")
    parser.add_argument("--skip_seconds", type=int, default=0,
                      help="Seconds to skip from bag start")
    
    # Add CSV output argument
    parser.add_argument("--csv_output", type=str,
                      help="Path to CSV file for storing grid search results")
    
    # Add metric selection argument
    parser.add_argument("--metrics", nargs='+', 
                      choices=['relational', 'mme', 'mpv', 'mom'],
                      default=['relational'],
                      help="Metrics to evaluate (default: relational)")
    
    args = parser.parse_args()
    
    # Parse parameter grid from JSON
    import json
    parameter_grid = json.loads(args.parameter_grid)
    
    optimizer = ParameterOptimizer(
        bag_filename=args.bag_filename,
        config_dir=args.config_dir,
        config_basename=args.config_basename,
        points2_topic=args.points2_topic,
        imu_topic=args.imu_topic,
        parameter_grid=parameter_grid,
        metrics=args.metrics,
        min_covered_distance=args.min_covered_distance,
        outlier_threshold_meters=args.outlier_threshold_meters,
        outlier_threshold_radians=args.outlier_threshold_radians,
        skip_seconds=args.skip_seconds,
        csv_output=args.csv_output
    )
    
    results = optimizer.find_best_parameters()
    
    print("\nBest parameters for each metric:")
    for result in results:
        print(f"\n{result.metric_name}:")
        print(f"  Best value: {result.best_value:.5f} ± {result.best_uncertainty:.5f} {result.unit}")
        print("  Parameters:")
        for param, value in result.best_parameters.items():
            print(f"    {param}: {value}")

if __name__ == "__main__":
    main() 