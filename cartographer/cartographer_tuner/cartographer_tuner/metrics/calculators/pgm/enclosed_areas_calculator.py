import numpy as np
import cv2
from typing import Dict, List, Optional, Union

from cartographer_tuner.metrics.metric import Metric
from cartographer_tuner.metrics.calculators.pgm.base_pgm_metric_calculator import BasePgmMetricCalculator
from cartographer_tuner.utils.visualization import show_image

__all__ = ["EnclosedAreasCalculator"]

class EnclosedAreasCalculator(BasePgmMetricCalculator):
    ENLOSED_AREAS = "enclosed_areas_count"
    METRIC_NAMES = [ENLOSED_AREAS]

    def __init__(
        self,
        map_data: Union[str, np.ndarray],
        min_area_percentage: int = 0.01,
        debug: bool = False,
        **kwargs,
    ):
        super().__init__(map_data)
        self.debug = debug
        self._min_area_percentage = min_area_percentage
        self._enclosed_count = None
        self._enclosed_contours = None

    @property
    def enclosed_contours(self) -> Optional[List[np.ndarray]]:
        return self._enclosed_contours
    
    def _get_unknown_mask(self):
        intensities = self.map_data.reshape(-1, 1).astype(np.float32) / 255.0
        criteria = (cv2.TERM_CRITERIA_EPS, None, 10)
        attempts = 10
        _, labels, centers = cv2.kmeans(intensities, 3, None, criteria, attempts, cv2.KMEANS_RANDOM_CENTERS)
        mid_idx = np.argsort(centers)[1]
        unknown_mask = (labels.flatten() == mid_idx).reshape(self.map_data.shape)
        return unknown_mask
    
    @staticmethod
    def _otsu_threshold(image):
        _, binary = cv2.threshold(
                image,
                0, 255,
                cv2.THRESH_BINARY + cv2.THRESH_OTSU
            )
        return binary
    

    def _find_enclosed_areas(self, image: np.ndarray):
        contours, _ = cv2.findContours(image, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        unoccupied_areas = []

        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            bounding_area = w * h
            
            if bounding_area < image.size * self._min_area_percentage:
                continue

            unoccupied_areas.append(contour)

        return unoccupied_areas
    
    def _calculate_contours(self):
        binary_map = self._otsu_threshold(self.map_data)
        if self.debug:
            show_image(binary_map, f"Binary map with")

        enclosed_contrours = self._find_enclosed_areas(binary_map)
        if self.debug:
            show_image(self.draw_contours(binary_map, enclosed_contrours), "Contours detected")
        self._enclosed_contours = enclosed_contrours

    def calculate(self, metrics: Optional[List[str]] = None) -> Dict[str, Metric]:
        metrics = self._process_metric_names(metrics)

        results = {}
        if EnclosedAreasCalculator.ENLOSED_AREAS in metrics:
            if self._enclosed_contours is None:
                self._calculate_contours()

            results[EnclosedAreasCalculator.ENLOSED_AREAS] = Metric(
                name=EnclosedAreasCalculator.ENLOSED_AREAS,
                value=len(self._enclosed_contours),
            )
        return results
    
    def debug_image(self):
        if self._enclosed_contours is None:
            self._calculate_contours()
        debug_img = cv2.normalize(self.map_data, None, alpha=0, beta=255 // 2, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        debug_img = self.draw_contours(debug_img, self._enclosed_contours)
        return debug_img    

    @staticmethod
    def draw_contours(img, contours):
        rgb_image = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
        colors = [(255, 0, 0), (0, 0, 255), (0, 255, 0), (255, 255, 0), (255, 0, 255), (0, 255, 255)]

        for i, contour in enumerate(contours):
            color = colors[i % len(colors)]
            cv2.drawContours(rgb_image, [contour], 0, color, 2)

        return rgb_image
