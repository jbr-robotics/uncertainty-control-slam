from pathlib import Path
from typing import Optional

from cartographer_tuner.metrics.calculators.base_metric_calculator import BaseMetricCalculator
from cartographer_tuner.metrics.calculators.exceptions import CalculatorFileNotFoundException

__all__ = ["BaseLuaMetricCalculator"]

class BaseLuaMetricCalculator(BaseMetricCalculator):
    """Abstract base class for metrics that evaluate Lua configurations.
    """
    
    def __init__(
        self,
        bag_filename: str,
        config_dir: str,
        config_basename: str,
        skip_seconds: int = 0,
        tmp_dir: Optional[str] = None
    ):
        self.bag_filename = Path(bag_filename)
        self.config_dir = Path(config_dir)
        self.config_basename = config_basename
        self.skip_seconds = skip_seconds
        
        if not self.bag_filename.exists():
            raise CalculatorFileNotFoundException(f"Bag file not found: {self.bag_filename}")
        if not self.config_dir.exists():
            raise CalculatorFileNotFoundException(f"Config directory not found: {self.config_dir}")
        if not (self.config_dir / self.config_basename).exists():
            raise CalculatorFileNotFoundException(f"Config file not found: {self.config_dir / self.config_basename}")
        
        self.tmp_dir = Path(tmp_dir) if tmp_dir else None
