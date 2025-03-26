from abc import abstractmethod
from pathlib import Path
from typing import Dict, List, Optional

import open3d as o3d

from ..base_calculator import BaseMetricCalculator, Metric

class BasePlyMetricCalculator(BaseMetricCalculator):
    """Abstract base class for metrics that evaluate PLY point cloud maps.
    
    This class extends the BaseMetricCalculator to provide a common interface
    for metrics that specifically evaluate the quality of point cloud maps
    in PLY format.
    
    All PLY metric calculators should have these common parameters:
    - map_path: Path to input PLY file
    """
    
    def __init__(self, map_path: str):
        """Initialize base PLY metric calculator.
        
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
            
        self.map_path = map_path
        self.point_cloud = o3d.io.read_point_cloud(str(map_path))
