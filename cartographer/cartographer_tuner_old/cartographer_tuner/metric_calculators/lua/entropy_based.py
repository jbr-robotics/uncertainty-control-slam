import os
import tempfile
from pathlib import Path
from typing import Dict, List, Optional

from .base import BaseLuaMetricCalculator
from ..base_calculator import Metric
from ..ply.entropy_based import EntropyMetricsCalculator as PlyEntropyMetricsCalculator

class EntropyMetricsCalculator(BaseLuaMetricCalculator):
    """Calculator for entropy-based metrics on Cartographer-generated maps.
    
    This calculator generates a map using Cartographer with the specified
    Lua configuration, exports it to PLY, and then uses the PLY-based entropy metrics to evaluate it.
    
    Available metrics:
    - mme: Mean Map Entropy
    - mpv: Mean Plane Variance
    - mom: Mutually Orthogonal Metric
    """
    
    def __init__(
        self,
        bag_filename: str,
        config_dir: str,
        config_basename: str,
        skip_seconds: int = 0,
        tmp_dir: Optional[str] = None,
        points2_topic: str = "/points2",
        imu_topic: str = "/imu"
    ):
        """Initialize entropy metrics calculator.
        
        Args:
            bag_filename: Path to input bag file
            config_dir: Directory containing Lua configs
            config_basename: Base name of config file
            skip_seconds: Seconds to skip from bag start
            tmp_dir: Optional directory for temporary files
            points2_topic: Topic name for point cloud data
            imu_topic: Topic name for IMU data
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
        self.points2_topic = points2_topic
        self.imu_topic = imu_topic
        
    @staticmethod
    def get_available_metrics() -> List[str]:
        """Get list of available metrics for this calculator.
        
        Returns:
            List of metric names that can be calculated
        """
        # Use the same metrics as the PLY-based calculator
        return PlyEntropyMetricsCalculator.get_available_metrics()
    
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
        
        try:
            # Generate map
            print("Generating SLAM map...")
            pbstream_path = self._generate_map()
            
            # Export to PLY
            print("Exporting map to PLY...")
            ply_path = self._export_to_ply(pbstream_path)
            
            # Calculate metrics using PLY-based calculator
            print("Calculating entropy metrics...")
            ply_calculator = PlyEntropyMetricsCalculator(str(ply_path))
            return ply_calculator.calculate(metrics)
            
        except Exception as e:
            print(f"Error calculating entropy metrics: {e}")
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
        # launcher_args["remap"] = [f"{self.points2_topic};/points2", f"{self.imu_topic};/imu"]
        
        # Create and run launcher
        launcher = OfflineCartographerLauncher(**launcher_args)
        launcher.run()
        
        return pbstream_path
    
    def _export_to_ply(self, pbstream_path: Path) -> Path:
        """Export map to PLY format using assets writer.
        
        Args:
            pbstream_path: Path to the pose graph file
            
        Returns:
            Path to the generated PLY file
        """
        from quality_estimator.launchers.assets_writer import AssetsWriterLauncher
        
        # Find assets writer configuration
        assets_writer_config = "assets_writer_ply.lua"
        assets_writer_config_path = self.config_dir / assets_writer_config
        
        # If the config doesn't exist in the provided directory, use the default one
        if not assets_writer_config_path.exists():
            # Default location in ROS2 installation
            assets_writer_config_path = Path("/opt/ros/humble/share/cartographer_ros/configuration_files/assets_writer_ply.lua")
            if not assets_writer_config_path.exists():
                raise FileNotFoundError(f"Assets writer configuration not found: {assets_writer_config}")
            assets_writer_config_dir = assets_writer_config_path.parent
            assets_writer_config = assets_writer_config_path.name
        else:
            assets_writer_config_dir = self.config_dir
        
        # Create launcher with required parameters
        launcher_args = {
            "configuration_directory": str(assets_writer_config_dir),
            "configuration_basename": assets_writer_config,
            "bag_filenames": str(self.bag_filename),
            "pose_graph_filename": str(pbstream_path)
        }
        
        # Create and run launcher
        launcher = AssetsWriterLauncher(**launcher_args)
        launcher.run()
        
        # Find the generated PLY file
        # The assets writer typically creates files in the current directory
        # with names based on the input bag file
        ply_files = list(Path.cwd().glob("*.ply"))
        if not ply_files:
            raise FileNotFoundError("No PLY files generated by assets writer")
        
        # Move the PLY file to our temporary directory
        ply_path = self.tmp_dir / ply_files[0].name
        ply_files[0].rename(ply_path)
        
        return ply_path


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Calculate entropy-based quality metrics for Cartographer maps")
    parser.add_argument("--bag_filename", required=True,
                      help="Path to input bag file")
    parser.add_argument("--config_dir", required=True,
                      help="Directory containing Lua configs")
    parser.add_argument("--config_basename", required=True,
                      help="Base name of config file")
    parser.add_argument("--points2_topic", default="/points2",
                      help="Topic name for point cloud data")
    parser.add_argument("--imu_topic", default="/imu",
                      help="Topic name for IMU data")
    parser.add_argument("--skip_seconds", type=int, default=0,
                      help="Seconds to skip from bag start")
    parser.add_argument("--tmp_dir", help="Directory for temporary files")
    parser.add_argument("--metrics", nargs='+', 
                      choices=['mme', 'mpv', 'mom'],
                      default=None,
                      help="Specific metrics to calculate (default: all)")
    
    args = parser.parse_args()
    
    calculator = EntropyMetricsCalculator(
        bag_filename=args.bag_filename,
        config_dir=args.config_dir,
        config_basename=args.config_basename,
        points2_topic=args.points2_topic,
        imu_topic=args.imu_topic,
        skip_seconds=args.skip_seconds,
        tmp_dir=args.tmp_dir
    )
    
    results = calculator.calculate(args.metrics)
    
    print("\nEntropy Metrics:")
    for name, metric in results.items():
        print(f"{name}: {metric.value:.5f} {metric.unit}")

if __name__ == "__main__":
    main()
