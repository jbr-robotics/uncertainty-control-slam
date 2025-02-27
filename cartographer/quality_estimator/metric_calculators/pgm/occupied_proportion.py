from typing import Dict, Any, List
import argparse
import sys
import json

import numpy as np

from .base import BasePgmMetricCalculator
from ...metric import Metric

class OccupiedProportion(BasePgmMetricCalculator):
    """Calculates the proportion of occupied cells in a PGM map.
    
    This metric measures what fraction of the map cells have values greater than
    the mean value, which typically indicates occupied or non-free space in
    occupancy grid maps.
    
    A higher proportion might indicate a more cluttered environment or a map
    with more detected obstacles.
    """

    @staticmethod
    def get_available_metrics() -> List[str]:
        """Get list of available metrics for this calculator."""
        return ["occupied_proportion"]
    
    def calculate(self) -> Dict[str, Metric]:
        """Calculate the occupied proportion metric.
        
        Returns:
            Dict containing the occupied_proportion metric
        """
        # Calculate the mean value of the map
        mean_value = np.mean(self.map_data)
        
        # Count cells with values greater than the mean
        occupied_cells = np.sum(self.map_data > mean_value)
        
        # Calculate the proportion
        total_cells = self.map_data.size
        proportion = occupied_cells / total_cells if total_cells > 0 else 0.0
        return {
            "occupied_proportion": Metric(
                name="occupied_proportion",
                value=float(proportion),
                uncertainty=0.0,  # No uncertainty calculation for this metric
                unit="fraction"
            )
        }

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
        "--output", 
        dest="output_file",
        help="Path to output JSON file (optional, defaults to stdout)"
    )
    
    args = parser.parse_args()
    
    try:
        # Create the metric calculator
        calculator = OccupiedProportion(
            map_path=args.pgm_file,
            yaml_path=args.yaml_file
        )
        
        # Calculate the metric
        metrics = calculator.calculate()

        print(metrics)
        
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
