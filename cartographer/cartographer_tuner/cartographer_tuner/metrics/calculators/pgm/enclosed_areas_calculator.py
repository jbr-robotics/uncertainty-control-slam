import numpy as np
import cv2
from typing import Dict, List, Optional

from cartographer_tuner.metrics.metric import Metric
from cartographer_tuner.metrics.calculators.pgm.base_pgm_metric_calculator import BasePgmMetricCalculator
from cartographer_tuner.utils.visualization import show_image

__all__ = ["EnclosedAreasCalculator"]

class EnclosedAreasCalculator(BasePgmMetricCalculator):
    ENLOSED_AREAS = "enclosed_areas_count"
    METRIC_NAMES = [ENLOSED_AREAS]

    def __init__(
        self,
        map_path: str,
        yaml_path: Optional[str] = None,
        undefined_value: int = 205,
        free_value: int = 0,
        occupied_value: int = 255,
        candidate_start: int = 255,
        candidate_end: int = 200,
        candidate_step: int = 5,
        debug: bool = False,
        **kwargs,
    ):
        super().__init__(map_path, yaml_path)
        self.undefined_value = undefined_value
        self.free_value = free_value
        self.occupied_value = occupied_value
        self.candidate_start = candidate_start
        self.candidate_end = candidate_end
        self.candidate_step = candidate_step
        self.debug = debug

        self._enclosed_count = None
        self._enclosed_contours = None

    @property
    def enclosed_contours(self) -> Optional[List[np.ndarray]]:
        return self._enclosed_contours

    def _count_enclosed_free_areas(self, binary_image: np.ndarray) -> int:
        inv = cv2.bitwise_not(binary_image)
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(inv, connectivity=8)
        enclosed = 0
        h, w = binary_image.shape

        for label in range(1, num_labels):
            x, y, w_box, h_box, area = stats[label]
            if x <= 0 or y <= 0 or (x + w_box) >= w or (y + h_box) >= h:
                continue
            enclosed += 1
        return enclosed

    def calculate(self, metrics: Optional[List[str]] = None) -> Dict[str, Metric]:
        metrics = self._process_metric_names(metrics)

        best_enclosed = 0

        undefined_mask = (self.map_data == self.undefined_value)

        for candidate in range(self.candidate_start, self.candidate_end - 1, -self.candidate_step):
            candidate_map = self.map_data.copy()
            candidate_map[undefined_mask] = candidate

            if self.debug:
                show_image(candidate_map, f"Candidate occupancy map (undefined set to {candidate})")

            ret, binary = cv2.threshold(
                candidate_map,
                0, 255,
                cv2.THRESH_BINARY + cv2.THRESH_OTSU
            )
            if self.debug:
                show_image(binary, f"Binary image (Otsu threshold={ret:.2f})")

            enclosed = self._count_enclosed_free_areas(binary)
            if self.debug:
                print(f"Candidate {candidate}: enclosed areas count = {enclosed}")

            if enclosed > best_enclosed:
                contours, hierarchy = cv2.findContours(
                    binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
                )
                self._enclosed_contours = contours
                best_enclosed = enclosed

        self._enclosed_count = best_enclosed

        results = {}
        if EnclosedAreasCalculator.ENLOSED_AREAS in metrics:
            results[EnclosedAreasCalculator.ENLOSED_AREAS] = Metric(
                name=EnclosedAreasCalculator.ENLOSED_AREAS,
                value=best_enclosed,
            )
        return results

