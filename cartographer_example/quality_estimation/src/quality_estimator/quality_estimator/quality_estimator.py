import os
import argparse
import tempfile
from pathlib import Path
from typing import Optional

from .config_manager import ConfigManager
from .offline_cartographer_launcher import OfflineCartographerLauncher
from .autogenerate_ground_truth_launcher import CartographerAutogenerateGroundTruthLauncher
from .cartographer_compute_relations_metrics_launcher import CartographerComputeRelationsMetricsLauncher

class QualityEstimator:
    def __init__(self,
                 bag_filename: str,
                 config_dir: str,
                 config_basename: str,
                 points2_topic: str,
                 imu_topic: str,
                 min_covered_distance: float = 20.0,
                 outlier_threshold_meters: float = 0.15,
                 outlier_threshold_radians: float = 0.02,
                 skip_seconds: int = 0,
                 tmp_dir: Optional[str] = None):
        """Initialize QualityEstimator.
        
        Args:
            bag_filename: Path to input bag file
            config_dir: Directory containing Lua configs
            config_basename: Base name of config file
            points2_topic: Topic name for point cloud data
            imu_topic: Topic name for IMU data
            min_covered_distance: Minimum distance for ground truth generation
            outlier_threshold_meters: Distance threshold for outliers
            outlier_threshold_radians: Angular threshold for outliers
            skip_seconds: Seconds to skip from bag start
            tmp_dir: Directory for temporary files (optional)
        """
        self.bag_filename = Path(bag_filename)
        self.config_dir = Path(config_dir)
        self.config_basename = config_basename
        self.points2_topic = points2_topic
        self.imu_topic = imu_topic
        self.min_covered_distance = min_covered_distance
        self.outlier_threshold_meters = outlier_threshold_meters
        self.outlier_threshold_radians = outlier_threshold_radians
        self.skip_seconds = skip_seconds
        
        # Create temporary directory if not provided
        if tmp_dir is None:
            self.tmp_dir = Path(tempfile.mkdtemp(prefix="cartographer_quality_"))
        else:
            self.tmp_dir = Path(tmp_dir)
            self.tmp_dir.mkdir(parents=True, exist_ok=True)
            
        # Initialize config manager
        self.config_manager = ConfigManager()
        
    def generate_optimized_map(self) -> Path:
        """Generate optimized map using original configuration.
        
        Returns:
            Path to generated .pbstream file
        """
        output_path = self.tmp_dir / "optimized.pbstream"
        
        launcher = (OfflineCartographerLauncher()
            .set_skip_seconds(self.skip_seconds)
            .set_no_rviz("true")
            .set_bag_filenames(str(self.bag_filename))
            .set_configuration_directory(str(self.config_dir))
            .set_configuration_basenames(self.config_basename)
            .set_points2_topic(self.points2_topic)
            .set_imu_topic(self.imu_topic)
            .set_save_state_filename(str(output_path)))
        
        launcher.run()
        return output_path
        
    def generate_ground_truth(self, optimized_map_path: Path) -> Path:
        """Generate ground truth relations from optimized map.
        
        Args:
            optimized_map_path: Path to optimized .pbstream file
            
        Returns:
            Path to generated relations file
        """
        output_path = self.tmp_dir / "relations.pbstream"
        
        launcher = (CartographerAutogenerateGroundTruthLauncher()
            .set_pose_graph_filename(str(optimized_map_path))
            .set_output_filename(str(output_path))
            .set_min_covered_distance(self.min_covered_distance)
            .set_outlier_threshold_meters(self.outlier_threshold_meters)
            .set_outlier_threshold_radians(self.outlier_threshold_radians))
            
        launcher.run()
        return output_path

    def generate_unoptimized_map(self) -> Path:
        """Generate unoptimized map with optimization disabled.
        
        Returns:
            Path to generated .pbstream file
        """
        # Load and modify configuration
        self.config_manager.load_config(
            self.config_dir / self.config_basename
        )
        
        # Disable optimization
        self.config_manager["map_builder.pose_graph.optimize_every_n_nodes"] = 0
        
        # Save modified config
        modified_config_path = self.tmp_dir / "config_no_optimization.lua"
        with open(modified_config_path, "w") as f:
            f.write(self.config_manager.to_lua_string("options"))
            
        output_path = self.tmp_dir / "unoptimized.pbstream"
        
        launcher = (OfflineCartographerLauncher()
            .set_skip_seconds(self.skip_seconds)
            .set_no_rviz("true")
            .set_bag_filenames(str(self.bag_filename))
            .set_configuration_directory(str(self.tmp_dir))
            .set_configuration_basenames(modified_config_path.name)
            .set_points2_topic(self.points2_topic)
            .set_imu_topic(self.imu_topic)
            .set_save_state_filename(str(output_path)))
            
        launcher.run()
        return output_path

    def compute_metrics(self, relations_path: Path, unoptimized_map_path: Path):
        """Compute quality metrics comparing unoptimized map to ground truth.
        
        Args:
            relations_path: Path to ground truth relations
            unoptimized_map_path: Path to unoptimized map
        """
        launcher = (CartographerComputeRelationsMetricsLauncher()
            .set_relations_filename(str(relations_path))
            .set_pose_graph_filename(str(unoptimized_map_path)))
            
        launcher.run()

    def run(self):
        """Execute full quality estimation pipeline."""
        prefix = self.__class__.__name__    
        print(f"{prefix}: Starting quality estimation...")
        print(f"{prefix}: Temporary directory: {self.tmp_dir}")
        try:
            print(f"{prefix}: Generating optimized map...")
            optimized_map = self.generate_optimized_map()
            
            print(f"{prefix}: Generating ground truth relations...")
            relations = self.generate_ground_truth(optimized_map)
            
            print(f"{prefix}: Generating unoptimized map...")
            unoptimized_map = self.generate_unoptimized_map()
            
            print(f"{prefix}: Computing quality metrics...")
            self.compute_metrics(relations, unoptimized_map)
            
        except Exception as e:
            print(f"{prefix}: Error during quality estimation: {str(e)}")
            raise

def main():
    parser = argparse.ArgumentParser(description="Estimate Cartographer SLAM quality")
    
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
    parser.add_argument("--min_covered_distance", type=float, default=20.0,
                      help="Minimum distance for ground truth generation")
    parser.add_argument("--outlier_threshold_meters", type=float, default=0.15,
                      help="Distance threshold for outliers")
    parser.add_argument("--outlier_threshold_radians", type=float, default=0.02,
                      help="Angular threshold for outliers")
    parser.add_argument("--skip_seconds", type=int, default=0,
                      help="Seconds to skip from bag start")
    parser.add_argument("--tmp_dir", help="Directory for temporary files")
    
    args = parser.parse_args()
    
    estimator = QualityEstimator(
        bag_filename=args.bag_filename,
        config_dir=args.config_dir,
        config_basename=args.config_basename,
        points2_topic=args.points2_topic,
        imu_topic=args.imu_topic,
        min_covered_distance=args.min_covered_distance,
        outlier_threshold_meters=args.outlier_threshold_meters,
        outlier_threshold_radians=args.outlier_threshold_radians,
        skip_seconds=args.skip_seconds,
        tmp_dir=args.tmp_dir
    )
    
    estimator.run()

if __name__ == "__main__":
    main()
