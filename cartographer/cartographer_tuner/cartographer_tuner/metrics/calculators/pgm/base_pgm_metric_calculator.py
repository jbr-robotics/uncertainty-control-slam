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
from abc import ABC, abstractmethod


__all__ = ["BasePgmMetricCalculator"]

class BasePgmMetricCalculator(BaseMetricCalculator, ABC):
    """Abstract base class for metrics that evaluate PGM map files.
    """

    @abstractmethod
    def debug_image(self) -> np.ndarray:
        raise NotImplementedError()
    
    def __init__(self, map_data: Union[str, np.ndarray]):
        """
        Initialize with either a file path to a PGM map or a numpy array.
        """
        if isinstance(map_data, str):
            self.map_path = Path(map_data)
            self._verify_map_path(self.map_path)
            self.map_data = self._load_map(self.map_path)
        elif isinstance(map_data, np.ndarray):
            self.map_data = map_data
        else:
            raise ValueError("map_data must be a file path or a numpy array")
        
        self.metadata = None
    
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
