import warnings
warnings.simplefilter("ignore", UserWarning)

from typing import Dict
import numpy as np
import open3d as o3d
from pathlib import Path
from map_metrics import mme, mpv, mom
from map_metrics.config import LidarConfig

class MapMetricsCalculator:
    """Calculate map quality metrics for PLY files."""
    
    def __init__(self, map_path: str):
        """Initialize calculator with a PLY map.
        
        Args:
            map_path: Path to the PLY file
            
        Raises:
            ValueError: If file doesn't exist or isn't a PLY file
        """
        map_path = Path(map_path)
        if not map_path.exists():
            raise ValueError(f"Map file not found: {map_path}")
        if map_path.suffix.lower() != '.ply':
            raise ValueError(f"Expected .ply file, got: {map_path}")
            
        self.point_cloud = o3d.io.read_point_cloud(str(map_path))
        
    def calculate_mme(self) -> float:
        """Calculate Mean Map Entropy for the loaded map.
        
        Returns:
            Mean Map Entropy value
        """
        return mme([self.point_cloud], [np.eye(4)], config=LidarConfig)
    
    def calculate_mpv(self) -> float:
        """Calculate Mean Plane Variance for the loaded map.
        
        Returns:
            Mean Plane Variance value
        """
        return mpv([self.point_cloud], [np.eye(4)], config=LidarConfig)
    
    def calculate_mom(self) -> float:
        """Calculate Mutually Orthogonal Metric for the loaded map.
        
        Returns:
            Mutually Orthogonal Metric value
        """
        return mom([self.point_cloud], [np.eye(4)], config=LidarConfig)
    
    def calculate_all(self) -> Dict[str, float]:
        """Calculate all metrics for the loaded map.
        
        Returns:
            Dictionary containing all metric values
        """
        metrics = {}
        
        try:
            metrics['mean_map_entropy'] = self.calculate_mme()
        except Exception as e:
            print(f"Warning: Failed to calculate MME: {e}")
            metrics['mean_map_entropy'] = float('nan')
            
        try:
            metrics['mean_plane_variance'] = self.calculate_mpv()
        except Exception as e:
            print(f"Warning: Failed to calculate MPV: {e}")
            metrics['mean_plane_variance'] = float('nan')
            
        try:
            metrics['mutually_orthogonal_metric'] = self.calculate_mom()
        except Exception as e:
            print(f"Warning: Failed to calculate MOM: {e}")
            metrics['mutually_orthogonal_metric'] = float('nan')
            
        return metrics

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Calculate quality metrics for PLY maps")
    parser.add_argument("map_path", help="Path to input PLY map file")
    parser.add_argument("--metric", choices=['mme', 'mpv', 'mom', 'all'], default='all',
                      help="Specific metric to calculate (default: all)")
    
    args = parser.parse_args()
    
    calculator = MapMetricsCalculator(args.map_path)
    
    if args.metric == 'mme':
        value = calculator.calculate_mme()
        print(f"Mean Map Entropy: {value:.5f}")
    elif args.metric == 'mpv':
        value = calculator.calculate_mpv()
        print(f"Mean Plane Variance: {value:.5f}")
    elif args.metric == 'mom':
        value = calculator.calculate_mom()
        print(f"Mutually Orthogonal Metric: {value:.5f}")
    else:
        metrics = calculator.calculate_all()
        print("\nMap Quality Metrics:")
        for name, value in metrics.items():
            print(f"{name}: {value:.5f}")

if __name__ == "__main__":
    main()