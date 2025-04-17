import numpy as np
import cv2
from typing import Dict, List, Optional, Tuple, Union

from cartographer_tuner.metrics.calculators.pgm.base_pgm_metric_calculator import BasePgmMetricCalculator
from cartographer_tuner.metrics.metric import Metric
from cartographer_tuner.utils.visualization import show_image

class CornerCountCalculator(BasePgmMetricCalculator):
    METRIC_NAMES = ["corner_count"]
    
    def __init__(
        self,
        map_data: Union[str, np.ndarray],
        precision: float = 0.01,
        min_distance: int = 10,
        debug: bool = False,
        **kwargs,
    ):
        """Initialize the corner count calculator.
        
        Args:
            map_path: Path to the PGM map file
            precision: Precision for corner detection
            min_distance: Minimum distance between corners
            debug: Whether to show intermediate images during processing
        """
        super().__init__(map_data)

        self._precision = precision
        self._min_distance = min_distance
        
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
    
    def _filter_close_corners(self, corners):
        filtered = []
        buckets = set()
        for coord in corners:
            coord_bucket = (coord[0] // self._min_distance, coord[1] // self._min_distance) 
            if coord_bucket in buckets:
                continue
            filtered.append(coord)
            buckets.add(coord_bucket)
        return filtered

    def _detect_corners(self, map_data: np.ndarray) -> List[Tuple[int, int]]:
        _, binary = cv2.threshold(
                map_data,
                0, 255,
                cv2.THRESH_BINARY + cv2.THRESH_OTSU
            )
        
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        corners = []
        for cnt in contours:
            epsilon = self._precision * cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, epsilon, True)
            corners.extend(approx)

        if corners is not None:
            corners = np.intp(corners).reshape(-1, 2)
            corners = [(pt[1], pt[0]) for pt in corners]
        else:
            corners = []
        
        filtered = self._filter_close_corners(corners)
        self._corners = filtered

        return self._corners

    def debug_image(self):
        if self._corners is None:
            self._corners = self._detect_corners(self.map_data)

        if len(self.map_data.shape) == 2:
            debug_img = cv2.cvtColor(self.map_data, cv2.COLOR_GRAY2BGR)
        else:
            debug_img = self.map_data.copy()

        for y, x in self._corners:
            cv2.circle(debug_img, (x, y), radius=2, color=(255, 0, 0), thickness=-1)

        return debug_img