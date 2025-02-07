import os
import tempfile
from pathlib import Path
from typing import Dict, Optional, List
import shutil

from .offline_cartographer_launcher import OfflineCartographerLauncher
from .assets_writer_launcher import CartographerAssetsWriterLauncher
from .map_metrics_calculator import MapMetricsCalculator

class MapMetricEstimator:
    def __init__(self,
                 bag_filename: str,
                 config_dir: str,
                 config_basename: str,
                 points2_topic: str,
                 imu_topic: str,
                 metrics: List[str],
                 skip_seconds: int = 0,
                 tmp_dir: Optional[str] = None):
        """Initialize MapMetricEstimator."""
        self.bag_filename = Path(bag_filename)
        self.config_dir = Path(config_dir)
        self.config_basename = config_basename
        self.points2_topic = points2_topic
        self.imu_topic = imu_topic
        self.metrics = metrics
        self.skip_seconds = skip_seconds
        
        self.tmp_dir = Path(tmp_dir) if tmp_dir else Path(tempfile.mkdtemp(prefix="cartographer_map_metrics_"))
        self.tmp_dir.mkdir(parents=True, exist_ok=True)
            
    def generate_map(self) -> Path:
        """Generate map using offline Cartographer."""
        pbstream_path = self.tmp_dir / "map.pbstream"
        
        launcher = (OfflineCartographerLauncher()
            .set_skip_seconds(self.skip_seconds)
            .set_no_rviz("true")
            .set_bag_filenames(str(self.bag_filename))
            .set_configuration_directory(str(self.config_dir))
            .set_configuration_basenames(self.config_basename)
            .set_points2_topic(self.points2_topic)
            .set_imu_topic(self.imu_topic)
            .set_save_state_filename(str(pbstream_path)))
        
        launcher.run()
        return pbstream_path
        
    def export_ply(self, pbstream_path: Path) -> Path:
        """Export .pbstream to .ply format."""
        # Get the path where assets writer will create the PLY file
        expected_ply_path = self.bag_filename.with_name(self.bag_filename.stem + "_map.ply")
        
        launcher = (CartographerAssetsWriterLauncher()
            .set_configuration_directory(str(self.config_dir))
            .set_config_file("assets_writer_ply.lua")
            .set_bag_filenames(str(self.bag_filename))
            .set_pose_graph_filename(str(pbstream_path)))
            
        launcher.run()
        
        # Move the PLY file to our temporary directory
        tmp_ply_path = self.tmp_dir / expected_ply_path.name
        shutil.move(str(expected_ply_path), str(tmp_ply_path))
        
        return tmp_ply_path
        
    def calculate_metrics(self, ply_path: Path) -> Dict[str, float]:
        """Calculate requested quality metrics for the PLY map."""
        calculator = MapMetricsCalculator(str(ply_path))
        results = {}
        
        metric_funcs = {
            'mme': calculator.calculate_mme,
            'mpv': calculator.calculate_mpv,
            'mom': calculator.calculate_mom
        }
        
        for metric in self.metrics:
            if metric not in metric_funcs:
                print(f"Warning: Unknown metric '{metric}', skipping...")
                continue
                
            try:
                results[metric] = metric_funcs[metric]()
            except Exception as e:
                print(f"Warning: Failed to calculate {metric}: {e}")
            
        return results

    def run(self) -> Dict[str, float]:
        """Execute full map metric estimation pipeline."""
        prefix = self.__class__.__name__
        print(f"{prefix}: Starting map metric estimation...")
        print(f"{prefix}: Temporary directory: {self.tmp_dir}")
        
        try:
            print(f"{prefix}: Generating map...")
            pbstream_path = self.generate_map()
            
            print(f"{prefix}: Exporting to PLY...")
            ply_path = self.export_ply(pbstream_path)
            
            print(f"{prefix}: Computing map metrics...")
            metrics = self.calculate_metrics(ply_path)
            
            # Print metrics summary
            print(f"\n{prefix}: Map Metrics Summary:")
            for metric, value in metrics.items():
                print(f"{prefix}: {metric}: {value:.5f}")
            
            return metrics
            
        except Exception as e:
            print(f"{prefix}: Error during map metric estimation: {str(e)}")
            raise

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="Estimate map quality metrics")
    
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
    parser.add_argument("--metrics", nargs='+', choices=['mme', 'mpv', 'mom'],
                      default=['mme', 'mpv', 'mom'],
                      help="Metrics to calculate (default: all)")
    parser.add_argument("--skip_seconds", type=int, default=0,
                      help="Seconds to skip from bag start")
    parser.add_argument("--tmp_dir", help="Directory for temporary files")
    
    args = parser.parse_args()
    
    estimator = MapMetricEstimator(
        bag_filename=args.bag_filename,
        config_dir=args.config_dir,
        config_basename=args.config_basename,
        points2_topic=args.points2_topic,
        imu_topic=args.imu_topic,
        metrics=args.metrics,
        skip_seconds=args.skip_seconds,
        tmp_dir=args.tmp_dir
    )
    
    metrics = estimator.run()
    
    # Print results
    print("\nResults:")
    for metric, value in metrics.items():
        print(f"{metric}: {value:.5f}")

if __name__ == "__main__":
    main()