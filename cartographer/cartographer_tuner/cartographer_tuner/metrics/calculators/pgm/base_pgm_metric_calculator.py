from pathlib import Path
from typing import Optional, Dict, Union

import numpy as np
import yaml
import cv2

from cartographer_tuner.metrics.calculators.base_metric_calculator import BaseMetricCalculator
from cartographer_tuner.metrics.calculators.exceptions import (
    CalculatorFileNotFoundException,
    CalculatorFileFormatException
)


__all__ = ["BasePgmMetricCalculator"]

class BasePgmMetricCalculator(BaseMetricCalculator):
    """Abstract base class for metrics that evaluate PGM map files.
    """
    
    def __init__(
        self, 
        map_path: Union[str, Path], 
        yaml_path: Optional[Union[str, Path]] = None
    ):
        """Initialize base PGM metric calculator.
        
        Args:
            map_path: Path to the PGM map file
            yaml_path: Optional path to the corresponding YAML metadata file
            
        Raises:
            CalculatorFileNotFoundException: If file doesn't exist or isn't a PGM file
            CalculatorFileFormatException: If YAML metadata file is specified but doesn't exist
        """
        self.map_path = Path(map_path)
        self._verify_map_path(self.map_path)
        self.map_data = self._load_map(self.map_path)
        
        self.metadata = None
        if yaml_path:
            self.yaml_path = Path(yaml_path)
            self._verify_yaml_path(self.yaml_path)
            self.metadata = self._load_yaml_metadata(self.yaml_path)
    
    @staticmethod
    def _invert_map(map_data: np.ndarray) -> np.ndarray:
        return 255 - map_data
    
    @staticmethod
    def _load_map(map_path: Path) -> np.ndarray:
        if not map_path.exists():
            raise CalculatorFileNotFoundException(f"Map file not found: {map_path}")
        if map_path.suffix.lower() != '.pgm':
            raise CalculatorFileFormatException(f"Expected .pgm file, got: {map_path}")

        map_data = cv2.imread(str(map_path), cv2.IMREAD_UNCHANGED)
        
        if map_data is None:
            raise CalculatorFileFormatException(f"Failed to load PGM file: {map_path}")
        
        map_data = BasePgmMetricCalculator._invert_map(map_data)

        return map_data
    
    @staticmethod
    def _load_yaml_metadata(yaml_path: Path) -> Optional[Dict]:
        if not yaml_path.exists():
            raise CalculatorFileNotFoundException(f"YAML metadata file not found: {yaml_path}")

        with open(yaml_path, 'r') as f:
            metadata = yaml.safe_load(f)

        if metadata is None:
            raise CalculatorFileFormatException(f"Failed to load YAML metadata: {yaml_path}")

        return metadata
    
    @staticmethod
    def _verify_map_path(map_path: Path) -> None:
        if not map_path.exists():
            raise CalculatorFileNotFoundException(f"Map file not found: {map_path}")
        if map_path.suffix.lower() != '.pgm':
            raise CalculatorFileFormatException(f"Expected .pgm file, got: {map_path}")
        
    @staticmethod
    def _verify_yaml_path(yaml_path: Path) -> None:
        if not yaml_path.exists():
            raise CalculatorFileNotFoundException(f"YAML metadata file not found: {yaml_path}")
