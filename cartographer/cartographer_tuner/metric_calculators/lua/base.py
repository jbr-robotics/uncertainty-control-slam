from abc import abstractmethod
from pathlib import Path
from typing import Dict, List, Optional, Any, Set, Union

from ..base_calculator import BaseMetricCalculator, Metric

class BaseLuaMetricCalculator(BaseMetricCalculator):
    """Abstract base class for metrics that evaluate Lua configurations.
    
    This class extends the BaseMetricCalculator to provide a common interface
    for metrics that specifically evaluate the quality of Lua configurations
    used in SLAM algorithms.
    
    All Lua metric calculators should have these common parameters:
    - bag_filename: Path to input bag file
    - config_dir: Directory containing Lua configs
    - config_basename: Base name of config file
    - skip_seconds: Seconds to skip from bag start
    - tmp_dir: Optional directory for temporary files
    """
    
    def __init__(
        self,
        bag_filename: str,
        config_dir: str,
        config_basename: str,
        skip_seconds: int = 0,
        tmp_dir: Optional[str] = None
    ):
        """Initialize base Lua metric calculator.
        
        Args:
            bag_filename: Path to input bag file
            config_dir: Directory containing Lua configs
            config_basename: Base name of config file
            skip_seconds: Seconds to skip from bag start
            tmp_dir: Optional directory for temporary files
            
        Raises:
            ValueError: If required files don't exist
        """
        self.bag_filename = Path(bag_filename)
        self.config_dir = Path(config_dir)
        self.config_basename = config_basename
        self.skip_seconds = skip_seconds
        
        # Validate inputs
        if not self.bag_filename.exists():
            raise ValueError(f"Bag file not found: {self.bag_filename}")
        if not self.config_dir.exists():
            raise ValueError(f"Config directory not found: {self.config_dir}")
        if not (self.config_dir / self.config_basename).exists():
            raise ValueError(f"Config file not found: {self.config_dir / self.config_basename}")
            
        # Create temporary directory if needed
        import tempfile
        self.tmp_dir = Path(tmp_dir) if tmp_dir else Path(tempfile.mkdtemp(prefix="cartographer_metrics_"))
        self.tmp_dir.mkdir(parents=True, exist_ok=True)
