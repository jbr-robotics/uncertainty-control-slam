import warnings
warnings.simplefilter("ignore", UserWarning)

from typing import Dict, List, Optional
import numpy as np
from pathlib import Path
from map_metrics import mme, mpv, mom
from map_metrics.config import LidarConfig

from .base import BasePlyMetricCalculator
from ..base_calculator import Metric

class EntropyMetricsCalculator(BasePlyMetricCalculator):
    """Calculator for entropy-based map quality metrics.
    
    This calculator provides the following metrics:
    - mme: Mean Map Entropy
    - mpv: Mean Plane Variance
    - mom: Mutually Orthogonal Metric
    
    These metrics evaluate the quality of point cloud maps based on
    information theory and geometric properties.
    """
    
    @staticmethod
    def get_available_metrics() -> List[str]:
        """Get list of available metrics for this calculator.
        
        Returns:
            List of metric names that can be calculated
        """
        return ['mme', 'mpv', 'mom']
    
    def calculate(self, metrics: Optional[List[str]] = None) -> Dict[str, Metric]:
        """Calculate requested entropy metrics.
        
        Args:
            metrics: Optional list of specific metrics to calculate.
                    If None, calculate all available metrics.
        
        Returns:
            Dictionary mapping metric names to their results
            
        Raises:
            ValueError: If any requested metric is not available
        """
        # If no metrics specified, calculate all
        if metrics is None:
            metrics = self.get_available_metrics()
        else:
            # Validate requested metrics
            self.validate_metrics(metrics)
        
        results = {}
        
        # Calculate each requested metric
        for metric_name in metrics:
            try:
                if metric_name == 'mme':
                    value = self._calculate_mme()
                    unit = ''
                elif metric_name == 'mpv':
                    value = self._calculate_mpv()
                    unit = ''
                elif metric_name == 'mom':
                    value = self._calculate_mom()
                    unit = ''
                    
                results[metric_name] = Metric(
                    name=metric_name,
                    value=value,
                    uncertainty=0.0,  # Entropy metrics don't have uncertainty values
                    unit=unit
                )
            except Exception as e:
                print(f"Warning: Failed to calculate {metric_name}: {e}")
                results[metric_name] = Metric(
                    name=metric_name,
                    value=float('nan'),
                    uncertainty=0.0,
                    unit=''
                )
                
        return results
    
    def _calculate_mme(self) -> float:
        """Calculate Mean Map Entropy for the loaded map.
        
        Returns:
            Mean Map Entropy value
        """
        return mme([self.point_cloud], [np.eye(4)], config=LidarConfig)
    
    def _calculate_mpv(self) -> float:
        """Calculate Mean Plane Variance for the loaded map.
        
        Returns:
            Mean Plane Variance value
        """
        return mpv([self.point_cloud], [np.eye(4)], config=LidarConfig)
    
    def _calculate_mom(self) -> float:
        """Calculate Mutually Orthogonal Metric for the loaded map.
        
        Returns:
            Mutually Orthogonal Metric value
        """
        return mom([self.point_cloud], [np.eye(4)], config=LidarConfig)


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Calculate entropy-based quality metrics for PLY maps")
    parser.add_argument("map_path", help="Path to input PLY map file")
    parser.add_argument("--metrics", nargs='+', choices=['mme', 'mpv', 'mom'], default=None,
                      help="Specific metrics to calculate")
    
    args = parser.parse_args()
    
    calculator = EntropyMetricsCalculator(args.map_path)
    results = calculator.calculate(args.metrics)
    
    print("\nMap Quality Metrics:")
    for name, metric in results.items():
        print(f"{name}: {metric.value:.5f} {metric.unit}")

if __name__ == "__main__":
    main()
