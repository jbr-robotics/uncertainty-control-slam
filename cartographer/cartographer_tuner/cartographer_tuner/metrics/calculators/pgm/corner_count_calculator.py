import numpy as np
import cv2
from typing import Dict, List, Optional, Tuple

from cartographer_tuner.metrics.calculators.pgm.base_pgm_metric_calculator import BasePgmMetricCalculator
from cartographer_tuner.metrics.metric import Metric
from cartographer_tuner.utils.visualization import show_image

class CornerCountCalculator(BasePgmMetricCalculator):
    METRIC_NAMES = ["corner_count"]
    
    def __init__(
        self,
        map_path: str,
        yaml_path: Optional[str] = None,
        block_size: int = 2,
        ksize: int = 3,
        k: float = 0.04,
        threshold: float = 0.01,
        min_distance: int = 10,
        filter_size: int = 5,
        sigma: float = 1.0,
        min_blob_size: int = 5,
        max_corners: int = 10000,
        debug: bool = False,
        **kwargs,
    ):
        """Initialize the corner count calculator.
        
        Args:
            map_path: Path to the PGM map file
            yaml_path: Optional path to the corresponding YAML metadata file
            block_size: Block size for Harris corner detection
            ksize: Aperture parameter for Sobel operator in Harris corner detection
            k: Harris detector free parameter
            threshold: Threshold for corner detection (relative to max response)
            min_distance: Minimum distance between corners
            filter_size: Size of the Gaussian-Laplace filter
            sigma: Standard deviation for the Gaussian-Laplace filter
            min_blob_size: Minimum size of blobs to keep after filtering
            max_corners: Maximum number of corners to detect
            debug: Whether to show intermediate images during processing
        """
        super().__init__(map_path, yaml_path)
        
        self.filter_size = filter_size
        self.sigma = sigma
        self.block_size = block_size
        self.ksize = ksize
        self.k = k
        self.threshold = threshold
        self.min_distance = min_distance
        self.min_blob_size = min_blob_size
        self.max_corners = max_corners
        self.debug = debug
        self._corners = None

    def calculate(self, metrics: Optional[List[str]] = None) -> Dict[str, Metric]:
        metrics = self._process_metric_names(metrics)
        
        if self._corners is None:
            self._corners = self._detect_corners(self.map_data)
        
        results = {}
        
        if self.METRIC_NAMES[0] in metrics:
            results[self.METRIC_NAMES[0]] = Metric(
                name=self.METRIC_NAMES[0],
                value=len(self._corners),
            )
        
        return results
    
    @property
    def corners(self) -> Optional[List[Tuple[int, int]]]:
        return self._corners
    
    def _apply_gaussian_laplace(self, map_data: np.ndarray) -> np.ndarray:
        blurred = cv2.GaussianBlur(
            map_data, 
            (self.filter_size, self.filter_size), 
            self.sigma
        )
        if self.debug:
            show_image(blurred, "Blurred map")
        laplacian = cv2.Laplacian(blurred, cv2.CV_64F)
        return laplacian
    
    def _detect_corners(self, map_data: np.ndarray) -> List[Tuple[int, int]]:
        if self.debug:
            show_image(map_data, "Original map")
        
        processed_map = self._apply_gaussian_laplace(map_data)
        if self.debug:
            show_image(processed_map, "Gaussian-Laplace filtered map")
        
        corners = cv2.goodFeaturesToTrack(
            processed_map.astype(np.float32),
            maxCorners=self.max_corners,
            qualityLevel=self.threshold,
            minDistance=self.min_distance,
            blockSize=self.block_size,
            useHarrisDetector=True,
            k=self.k
        )
        
        if corners is not None:
            corners = np.intp(corners).reshape(-1, 2)
            self._corners = [(pt[1], pt[0]) for pt in corners]
        else:
            self._corners = []
        
        return self._corners
