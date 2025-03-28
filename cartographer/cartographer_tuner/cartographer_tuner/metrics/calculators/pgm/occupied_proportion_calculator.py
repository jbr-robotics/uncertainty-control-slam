from typing import Dict, List, Optional

import numpy as np

from cartographer_tuner.metrics.calculators.pgm.base_pgm_metric_calculator import BasePgmMetricCalculator
from cartographer_tuner.metrics.metric import Metric
from cartographer_tuner.utils.visualization import show_image

__all__ = ["OccupiedProportionCalculator"]

class OccupiedProportionCalculator(BasePgmMetricCalculator):
    OCCUPIED_PROPORTION = "occupied_proportion"
    METRIC_NAMES = [OCCUPIED_PROPORTION]

    def __init__(
        self,
        map_path: str,
        yaml_path: Optional[str] = None,
        debug: bool = False,
        **kwargs,
    ):
        super().__init__(map_path, yaml_path)
        self.debug = debug

    def calculate(self, metrics: Optional[List[str]] = None) -> Dict[str, Metric]:
        metrics = self._process_metric_names(metrics)
            
        results = {}
        
        if self.OCCUPIED_PROPORTION in metrics:
            mean_value = np.mean(self.map_data)

            if self.debug:
                show_image((self.map_data > mean_value).astype(np.uint8) * 255, "Occupied cells")
            
            
            occupied_cells = np.sum(self.map_data > mean_value)
            
            total_cells = self.map_data.size
            proportion = occupied_cells / total_cells if total_cells > 0 else 0.0
            
            results[self.OCCUPIED_PROPORTION] = Metric(
                name=self.OCCUPIED_PROPORTION,
                value=float(proportion),
            )
        
        return results

