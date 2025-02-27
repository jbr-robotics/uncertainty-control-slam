import os
import tempfile
from pathlib import Path
from typing import Dict, List, Optional, Union
import shutil
import subprocess

from .base import BaseLuaMetricCalculator, Metric

class RelationMetricsCalculator(BaseLuaMetricCalculator):
    """Calculator for relation-based metrics comparing trajectory against ground truth.
    
    This calculator provides metrics that evaluate the quality of SLAM results
    by comparing the estimated trajectory against ground truth data.
    
    Available metrics:
    - translational_error: Absolute translational error
    - rotational_error: Absolute rotational error
    """
    
    def __init__(
        self,
        bag_filename: str,
        config_dir: str,
        config_basename: str,
        min_covered_distance: float = 20.0,
        outlier_threshold_meters: float = 0.15,
        outlier_threshold_radians: float = 0.02,
        skip_seconds: int = 0,
        tmp_dir: Optional[str] = None
    ):
        """Initialize relation metrics calculator.
        
        Args:
            bag_filename: Path to input bag file
            config_dir: Directory containing Lua configs
            config_basename: Base name of config file
            min_covered_distance: Minimum distance for ground truth generation
            outlier_threshold_meters: Distance threshold for outliers
            outlier_threshold_radians: Angular threshold for outliers
            skip_seconds: Seconds to skip from bag start
            tmp_dir: Optional directory for temporary files
        """
        # Initialize base class
        super().__init__(
            bag_filename=bag_filename,
            config_dir=config_dir,
            config_basename=config_basename,
            skip_seconds=skip_seconds,
            tmp_dir=tmp_dir
        )
        
        # Store additional parameters
        self.min_covered_distance = min_covered_distance
        self.outlier_threshold_meters = outlier_threshold_meters
        self.outlier_threshold_radians = outlier_threshold_radians
        
    @staticmethod
    def get_available_metrics() -> List[str]:
        """Get list of available metrics for this calculator.
        
        Returns:
            List of metric names that can be calculated
        """
        return ['translational_error', 'rotational_error']
    
    def calculate(self, metrics: Optional[List[str]] = None) -> Dict[str, Metric]:
        """Calculate requested relation metrics.
        
        Args:
            metrics: Optional list of specific metrics to calculate.
                    If None, calculate all available metrics.
        
        Returns:
            Dictionary mapping metric names to their results
            
        Raises:
            ValueError: If any requested metric is not available
            RuntimeError: If calculation fails
        """
        # If no metrics specified, calculate all
        if metrics is None:
            metrics = self.get_available_metrics()
        else:
            # Validate requested metrics
            self.validate_metrics(metrics)
        
        try:
            # Generate map and ground truth
            print("Generating SLAM map...")
            pbstream_path = self._generate_map()
            
            print("Generating ground truth...")
            relations_path = self._generate_ground_truth(pbstream_path)
            
            print("Computing relation metrics...")
            raw_metrics = self._compute_relation_metrics(pbstream_path, relations_path)
            
            # Convert raw metrics to standardized format
            results = {}
            for metric_name in metrics:
                if metric_name == 'translational_error':
                    if 'Abs translational error' in raw_metrics:
                        data = raw_metrics['Abs translational error']
                        results[metric_name] = Metric(
                            name=metric_name,
                            value=data['value'],
                            uncertainty=data['uncertainty'],
                            unit=data['unit']
                        )
                elif metric_name == 'rotational_error':
                    if 'Abs rotational error' in raw_metrics:
                        data = raw_metrics['Abs rotational error']
                        results[metric_name] = Metric(
                            name=metric_name,
                            value=data['value'],
                            uncertainty=data['uncertainty'],
                            unit=data['unit']
                        )
            
            return results
            
        except Exception as e:
            print(f"Error calculating relation metrics: {e}")
            # Return NaN values for all requested metrics
            return {
                name: Metric(
                    name=name,
                    value=float('nan'),
                    uncertainty=float('nan'),
                    unit=''
                )
                for name in metrics
            }
    
    def _generate_map(self) -> Path:
        """Generate map using offline Cartographer.
        
        Returns:
            Path to the generated .pbstream file
        """
        from quality_estimator.launchers.offline_cartographer import OfflineCartographerLauncher
        
        pbstream_path = self.tmp_dir / "map.pbstream"
        
        # Create launcher with required parameters
        launcher_args = {
            "skip_seconds": self.skip_seconds,
            "no_rviz": "true",
            "bag_filenames": str(self.bag_filename),
            "configuration_directory": str(self.config_dir),
            "configuration_basenames": self.config_basename,
            "save_state_filename": str(pbstream_path)
        }
        
        # Add remappings if needed
        # launcher_args["remap"] = ["/scan;/base_scan", "/imu;/torso_lift_imu/data"]
        
        # Create and run launcher
        launcher = OfflineCartographerLauncher(**launcher_args)
        launcher.run()
        
        return pbstream_path
    
    def _generate_ground_truth(self, pbstream_path: Path) -> Path:
        """Generate ground truth relations from pose graph.
        
        Args:
            pbstream_path: Path to the pose graph file
            
        Returns:
            Path to the generated relations file
        """
        from quality_estimator.launchers.autogenerate_ground_truth import AutogenerateGroundTruthLauncher
        
        relations_path = self.tmp_dir / "ground_truth.relations"
        
        # Create launcher with required parameters
        launcher_args = {
            "pose_graph_filename": str(pbstream_path),
            "output_filename": str(relations_path),
            "min_covered_distance": self.min_covered_distance,
            "outlier_threshold_meters": self.outlier_threshold_meters,
            "outlier_threshold_radians": self.outlier_threshold_radians
        }
        
        # Create and run launcher
        launcher = AutogenerateGroundTruthLauncher(**launcher_args)
        launcher.run()
        
        return relations_path
    
    def _compute_relation_metrics(self, pbstream_path: Path, relations_path: Path) -> Dict[str, Dict]:
        """Compute relation metrics comparing pose graph against ground truth.
        
        Args:
            pbstream_path: Path to the pose graph file
            relations_path: Path to the relations file
            
        Returns:
            Dictionary containing metrics with values and uncertainties
        """
        from quality_estimator.launchers.cartographer_compute_relations_metrics import CartographerComputeRelationsMetricsLauncher
        
        # Create launcher with required parameters
        launcher_args = {
            "pose_graph_filename": str(pbstream_path),
            "relations_filename": str(relations_path)
        }
        
        # Create and run launcher
        launcher = CartographerComputeRelationsMetricsLauncher(**launcher_args)
        return launcher.run()


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Calculate relation-based quality metrics")
    parser.add_argument("--bag_filename", required=True,
                      help="Path to input bag file")
    parser.add_argument("--config_dir", required=True,
                      help="Directory containing Lua configs")
    parser.add_argument("--config_basename", required=True,
                      help="Base name of config file")
    parser.add_argument("--min_covered_distance", type=float, default=20.0,
                      help="Minimum distance for ground truth generation")
    parser.add_argument("--outlier_threshold_meters", type=float, default=0.15,
                      help="Distance threshold for outliers")
    parser.add_argument("--outlier_threshold_radians", type=float, default=0.02,
                      help="Angular threshold for outliers")
    parser.add_argument("--skip_seconds", type=int, default=0,
                      help="Seconds to skip from bag start")
    parser.add_argument("--tmp_dir", help="Directory for temporary files")
    parser.add_argument("--metrics", nargs='+', 
                      choices=['translational_error', 'rotational_error'],
                      default=None,
                      help="Specific metrics to calculate (default: all)")
    
    args = parser.parse_args()
    
    calculator = RelationMetricsCalculator(
        bag_filename=args.bag_filename,
        config_dir=args.config_dir,
        config_basename=args.config_basename,
        min_covered_distance=args.min_covered_distance,
        outlier_threshold_meters=args.outlier_threshold_meters,
        outlier_threshold_radians=args.outlier_threshold_radians,
        skip_seconds=args.skip_seconds,
        tmp_dir=args.tmp_dir
    )
    
    results = calculator.calculate(args.metrics)
    
    print("\nRelation Metrics:")
    for name, metric in results.items():
        print(f"{name}: {metric.value:.5f} ± {metric.uncertainty:.5f} {metric.unit}")

if __name__ == "__main__":
    main()
