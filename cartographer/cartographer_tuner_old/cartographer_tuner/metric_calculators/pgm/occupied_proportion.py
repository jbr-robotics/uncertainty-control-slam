from typing import Dict, Any, List, Optional
import argparse
import sys
import json

import numpy as np

from .base import BasePgmMetricCalculator, show_image
from ...metric import Metric

class OccupiedProportionCalculator(BasePgmMetricCalculator):
    """Calculates the proportion of occupied cells in a PGM map.
    
    This metric measures what fraction of the map cells have values greater than
    the mean value, which typically indicates occupied or non-free space in
    occupancy grid maps.
    
    A higher proportion might indicate a more cluttered environment or a map
    with more detected obstacles.
    """

    # Define available metrics
    OCCUPIED_PROPORTION = "occupied_proportion"

    def __init__(
        self,
        map_path: str,
        yaml_path: Optional[str] = None,
        debug: bool = False
    ):
        """
        Initialize the occupied proportion calculator.
        
        Args:
            map_path: Path to the PGM map file.
            yaml_path: Optional path to the corresponding YAML metadata file.
            debug: Whether to show intermediate images during processing.
        """
        super().__init__(map_path, yaml_path)
        self.debug = debug

    @staticmethod
    def get_available_metrics() -> List[str]:
        """Get list of available metrics for this calculator."""
        return ["occupied_proportion"]
    
    def calculate(self, metrics: Optional[List[str]] = None) -> Dict[str, Metric]:
        """Calculate the occupied proportion metric.
        
        Args:
            metrics: Optional list of specific metrics to calculate.
                    If None, calculate all available metrics.
        
        Returns:
            Dictionary mapping metric names to their results
        """
        # Validate requested metrics
        if metrics:
            self.validate_metrics(metrics)
        else:
            metrics = self.get_available_metrics()
            
        # Calculate metrics
        results = {}
        
        if self.OCCUPIED_PROPORTION in metrics:
            # Calculate the mean value of the map
            mean_value = np.mean(self.map_data)

            #################################################
            # print(f"Mean value: {mean_value}")
            # # Draw a histogram of the map data
            # hist, bins = np.histogram(self.map_data, bins=10)
            
            # # Find the maximum bar height for scaling
            # max_height = np.max(hist)
            
            # # Draw the histogram bars
            # for i in range(len(hist)):
            #     bar_height = int(hist[i] / max_height * 10)  # Scale to 10 characters tall
            #     bar = '█' * bar_height
            #     print(f"{bins[i]:6.2f} - {bins[i+1]:6.2f} | {bar}")

            if self.debug:
                show_image((self.map_data > mean_value).astype(np.uint8) * 255, "Occupied cells")
            
            
            occupied_cells = np.sum(self.map_data > mean_value)
            
            # Calculate the proportion
            total_cells = self.map_data.size
            proportion = occupied_cells / total_cells if total_cells > 0 else 0.0
            
            results[self.OCCUPIED_PROPORTION] = Metric(
                name=self.OCCUPIED_PROPORTION,
                value=float(proportion),
                uncertainty=0.0,  # No uncertainty calculation for this metric
                unit="fraction"
            )
        
        return results

def main():
    """Command line interface for calculating the occupied proportion metric."""
    parser = argparse.ArgumentParser(
        description="Calculate the proportion of occupied cells in a PGM map."
    )
    parser.add_argument(
        "pgm_file", 
        help="Path to the PGM map file"
    )
    parser.add_argument(
        "--yaml", 
        dest="yaml_file",
        help="Path to the corresponding YAML metadata file (optional)"
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Show intermediate images during processing"
    )
    
    args = parser.parse_args()
    
    try:
        # Create the metric calculator
        calculator = OccupiedProportionCalculator(
            map_path=args.pgm_file,
            yaml_path=args.yaml_file,
            debug=args.debug
        )
        
        # Calculate the metric
        metrics = calculator.calculate()

        print(metrics)
        
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
