import os
import tempfile
from pathlib import Path
from typing import Dict, List, Optional, Any, Type, TypeVar, Set

from ..base_calculator import BaseMetricCalculator, Metric
from ..pgm.corner_count import CornerCountCalculator
from ..pgm.enclosed_areas import EnclosedAreasCalculator
from ..pgm.occupied_proportion import OccupiedProportionCalculator
from .base import BaseLuaMetricCalculator
from quality_estimator.launchers.lua_to_pgm import LuaToPgmLauncher


class LuaPgmMetricCalculator(BaseLuaMetricCalculator):
    """Unified calculator for all PGM-based metrics using Lua configurations.
    
    This class combines all PGM-based metrics into a single calculator that
    generates the PGM map only once and then calculates all requested metrics.
    """
    
    # Define metric groups for better organization
    CORNER_METRICS = set(CornerCountCalculator.get_available_metrics())
    ENCLOSED_AREAS_METRICS = set(EnclosedAreasCalculator.get_available_metrics())
    OCCUPIED_PROPORTION_METRICS = set(OccupiedProportionCalculator.get_available_metrics())
    
    def __init__(
        self,
        bag_filename: str,
        config_dir: str,
        config_basename: str,
        skip_seconds: int = 0,
        tmp_dir: Optional[str] = None,
        resolution: float = 0.05,
        topic_remappings: Optional[List[str]] = None,
        pgm_metric_kwargs: Optional[Dict[str, Any]] = None,
        **kwargs
    ):
        """Initialize the unified PGM-based metric calculator."""
        # Initialize the base class
        super().__init__(
            bag_filename=bag_filename,
            config_dir=config_dir,
            config_basename=config_basename,
            skip_seconds=skip_seconds,
            tmp_dir=tmp_dir
        )
        
        # Store additional arguments
        self.pgm_metric_kwargs = pgm_metric_kwargs or {}
        self.resolution = resolution
        self.topic_remappings = topic_remappings or []
        
        # Initialize the launcher and PGM calculators to None (will be created when needed)
        self.launcher = None
        self.corner_calculator = None
        self.enclosed_areas_calculator = None
        self.occupied_proportion_calculator = None
    
    @staticmethod
    def get_available_metrics() -> List[str]:
        """Get list of all available metrics from all PGM calculators."""
        metrics = []
        metrics.extend(CornerCountCalculator.get_available_metrics())
        metrics.extend(EnclosedAreasCalculator.get_available_metrics())
        metrics.extend(OccupiedProportionCalculator.get_available_metrics())
        return metrics
    
    def _ensure_map_generated(self) -> None:
        """Ensure that the PGM map is generated."""
        if self.launcher is None:
            # Create a temporary directory for the map
            map_filestem = str(Path(self.tmp_dir) / Path(self.bag_filename).stem)
            
            # Create and run the launcher
            self.launcher = LuaToPgmLauncher(
                bag_filenames=str(self.bag_filename),
                configuration_directory=str(self.config_dir),
                configuration_basenames=str(self.config_basename),
                skip_seconds=self.skip_seconds,
                map_filestem=map_filestem,
                resolution=self.resolution,
                remap=self.topic_remappings
            )
            
            # Run the launcher - it will raise an exception if it fails
            self.launcher.run()
    
    def _ensure_calculator_initialized(self, metric: str) -> BaseMetricCalculator:
        """Ensure that the appropriate calculator for the given metric is initialized.
        
        Args:
            metric: The metric to initialize a calculator for
            
        Returns:
            The appropriate calculator for the given metric
            
        Raises:
            ValueError: If the metric is not recognized
        """
        # Ensure the map is generated first
        self._ensure_map_generated()
        
        # Get the PGM and YAML paths
        pgm_path = str(self.launcher.get_pgm_path())
        yaml_path = str(self.launcher.get_yaml_path())
        
        # Initialize the appropriate calculator based on the metric
        if metric in self.CORNER_METRICS:
            if self.corner_calculator is None:
                self.corner_calculator = CornerCountCalculator(
                    map_path=pgm_path,
                    yaml_path=yaml_path,
                    **self.pgm_metric_kwargs
                )
            return self.corner_calculator
            
        elif metric in self.ENCLOSED_AREAS_METRICS:
            if self.enclosed_areas_calculator is None:
                self.enclosed_areas_calculator = EnclosedAreasCalculator(
                    map_path=pgm_path,
                    yaml_path=yaml_path,
                    **self.pgm_metric_kwargs
                )
            return self.enclosed_areas_calculator
            
        elif metric in self.OCCUPIED_PROPORTION_METRICS:
            if self.occupied_proportion_calculator is None:
                self.occupied_proportion_calculator = OccupiedProportionCalculator(
                    map_path=pgm_path,
                    yaml_path=yaml_path,
                    **self.pgm_metric_kwargs
                )
            return self.occupied_proportion_calculator
            
        else:
            raise ValueError(f"Unknown metric: {metric}")
    
    def calculate(self, metrics: Optional[List[str]] = None) -> Dict[str, Metric]:
        """Calculate the requested metrics.
        
        Args:
            metrics: Optional list of specific metrics to calculate.
                    If None, calculate all available metrics.
        
        Returns:
            Dictionary mapping metric names to their results
        """
        # If no metrics specified, calculate all available metrics
        if metrics is None:
            metrics = self.get_available_metrics()
        else:
            # Validate requested metrics
            self.validate_metrics(metrics)
        
        # Group metrics by calculator to minimize calculator initialization
        corner_metrics = [m for m in metrics if m in self.CORNER_METRICS]
        enclosed_areas_metrics = [m for m in metrics if m in self.ENCLOSED_AREAS_METRICS]
        occupied_proportion_metrics = [m for m in metrics if m in self.OCCUPIED_PROPORTION_METRICS]
        
        # Calculate metrics and combine results
        results = {}
        
        # Generate the map only once
        self._ensure_map_generated()
        
        # Calculate corner metrics if requested
        if corner_metrics:
            calculator = self._ensure_calculator_initialized(corner_metrics[0])
            results.update(calculator.calculate(corner_metrics))
        
        # Calculate enclosed areas metrics if requested
        if enclosed_areas_metrics:
            calculator = self._ensure_calculator_initialized(enclosed_areas_metrics[0])
            results.update(calculator.calculate(enclosed_areas_metrics))
        
        # Calculate occupied proportion metrics if requested
        if occupied_proportion_metrics:
            calculator = self._ensure_calculator_initialized(occupied_proportion_metrics[0])
            results.update(calculator.calculate(occupied_proportion_metrics))
        
        return results


def main():
    """Main entry point for running PGM-based metrics from the command line.
    
    This function provides a command-line interface for running all PGM-based metrics
    on a ROS bag file using a Lua configuration.
    """
    import argparse
    
    # Create argument parser
    parser = argparse.ArgumentParser(
        description="Calculate PGM-based quality metrics for Cartographer maps"
    )
    
    # Required arguments
    parser.add_argument(
        "--bag_filename", 
        required=True,
        help="Path to input bag file"
    )
    parser.add_argument(
        "--config_dir", 
        required=True,
        help="Directory containing Lua configs"
    )
    parser.add_argument(
        "--config_basename", 
        required=True,
        help="Base name of config file"
    )
    
    # Optional arguments
    parser.add_argument(
        "--skip_seconds", 
        type=int, 
        default=0,
        help="Seconds to skip from bag start"
    )
    parser.add_argument(
        "--resolution", 
        type=float, 
        default=0.05,
        help="Resolution of the map in meters per pixel"
    )
    parser.add_argument(
        "--tmp_dir", 
        help="Directory for temporary files"
    )
    parser.add_argument(
        "--remap", 
        action="append",
        help="Topic remapping in format '/source_topic;/target_topic'. Can be specified multiple times."
    )
    parser.add_argument(
        "--metrics", 
        nargs='+',
        choices=LuaPgmMetricCalculator.get_available_metrics(),
        default=None,
        help="Specific metrics to calculate (default: all)"
    )
    
    # Parse arguments
    args = parser.parse_args()
    
    # Create unified calculator
    calculator = LuaPgmMetricCalculator(
        bag_filename=args.bag_filename,
        config_dir=args.config_dir,
        config_basename=args.config_basename,
        skip_seconds=args.skip_seconds,
        tmp_dir=args.tmp_dir,
        resolution=args.resolution,
        topic_remappings=args.remap
    )
    
    try:
        # Calculate metrics
        results = calculator.calculate(args.metrics)
        
        # Print results
        print("\nPGM-Based Metrics:")
        for name, metric in results.items():
            print(f"{name}: {metric.value:.5f} {metric.unit}")
            
    except Exception as e:
        print(f"Error calculating metrics: {e}")
    
    print("\nMetrics calculation complete.")


if __name__ == "__main__":
    main()
