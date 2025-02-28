import os
import sys
import json
import argparse
import numpy as np
import cv2
from typing import Dict, List, Optional

from ..base_calculator import Metric
from .base import BasePgmMetricCalculator

def show_image(image: np.ndarray, title: str = "Image", wait_key: bool = True):
    """Display an image in a window.
    
    Args:
        image: Image to display
        title: Window title
        wait_key: Whether to wait for a key press before continuing
    """
    cv2.imshow(title, image)
    if wait_key:
        cv2.waitKey(0)
        cv2.destroyWindow(title)

class EnclosedAreasCalculator(BasePgmMetricCalculator):
    """Calculator for counting enclosed free areas in PGM maps.
    
    The algorithm divides all cells into two categories: free and occupied/undefined.
    Undefined regions are initially remapped to the maximum occupancy value.
    The map is then thresholded using Otsu’s method so that obstacles (occupied/undefined)
    have the same value. Free areas completely enclosed by obstacles are then identified.
    The process is repeated, assigning undefined regions slightly lower occupancy values,
    and the maximum number of enclosed areas found during these iterations is returned.
    """

    ENLOSED_AREAS_COUNT = "enclosed_areas_count"

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
        debug: bool = False
    ):
        """
        Initialize the enclosed areas calculator.
        
        Args:
            map_path: Path to the PGM map file.
            yaml_path: Optional path to the corresponding YAML metadata file.
            undefined_value: Pixel value used in the map for undefined areas.
            free_value: Pixel value corresponding to free cells.
            occupied_value: Pixel value corresponding to definitely occupied cells.
            candidate_start: Starting occupancy value for undefined areas.
            candidate_end: Ending occupancy value (inclusive) for candidate iterations.
            candidate_step: Step size to decrease occupancy probability for undefined cells.
            debug: Whether to show intermediate images during processing.
        """
        super().__init__(map_path, yaml_path)
        self.undefined_value = undefined_value
        self.free_value = free_value
        self.occupied_value = occupied_value
        self.candidate_start = candidate_start
        self.candidate_end = candidate_end
        self.candidate_step = candidate_step
        self.debug = debug

        self._enclosed_count = None

    @staticmethod
    def get_available_metrics() -> List[str]:
        """Get list of available metrics for this calculator.
        
        Returns:
            List of metric names that can be calculated.
        """
        return [EnclosedAreasCalculator.ENLOSED_AREAS_COUNT]

    def _count_enclosed_free_areas(self, binary_image: np.ndarray) -> int:
        """
        Count free-space connected components that do not touch the border.
        Assumes binary_image has obstacles as 255 and free space as 0.
        
        Args:
            binary_image: Thresholded binary map.
        
        Returns:
            Number of enclosed free areas.
        """
        # Invert the image: free becomes 255, obstacles 0,
        # so we can use connectedComponents to label free regions.
        inv = cv2.bitwise_not(binary_image)
        # Get connected components; note: background (0) is not counted.
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(inv, connectivity=8)
        enclosed = 0
        h, w = binary_image.shape

        # Start from label 1 (skip background)
        for label in range(1, num_labels):
            x, y, w_box, h_box, area = stats[label]
            # If the bounding box touches the image border, skip it.
            if x <= 0 or y <= 0 or (x + w_box) >= w or (y + h_box) >= h:
                continue
            enclosed += 1
        return enclosed

    def calculate(self, metrics: Optional[List[str]] = None) -> Dict[str, Metric]:
        """Calculate enclosed area based metric.
        
        Iterates through candidate occupancy values for undefined areas, applies
        Otsu thresholding, counts the enclosed free areas, and returns the maximum count.
        
        Args:
            metrics: Optional list of specific metrics to calculate.
                     If None, calculate all available metrics.
        
        Returns:
            Dictionary mapping metric names to their results.
        """
        if metrics:
            self.validate_metrics(metrics)
        else:
            metrics = self.get_available_metrics()

        best_enclosed = 0

        # Create a mask for undefined areas based on the provided undefined_value.
        undefined_mask = (self.map_data == self.undefined_value)

        # Iterate candidate values from candidate_start down to candidate_end.
        for candidate in range(self.candidate_start, self.candidate_end - 1, -self.candidate_step):
            # Copy the map and assign candidate value to undefined areas.
            candidate_map = self.map_data.copy()
            candidate_map[undefined_mask] = candidate

            if self.debug:
                show_image(candidate_map, f"Candidate occupancy map (undefined set to {candidate})")

            # Apply Otsu's thresholding. We use THRESH_BINARY so that obstacles become 255.
            # Note: free cells (assumed to be free_value, typically 0) remain 0.
            ret, binary = cv2.threshold(
                candidate_map,
                0, 255,
                cv2.THRESH_BINARY + cv2.THRESH_OTSU
            )
            if self.debug:
                show_image(binary, f"Binary image (Otsu threshold={ret:.2f})")

            # Count enclosed free areas.
            enclosed = self._count_enclosed_free_areas(binary)
            if self.debug:
                print(f"Candidate {candidate}: enclosed areas count = {enclosed}")

            best_enclosed = max(best_enclosed, enclosed)

        self._enclosed_count = best_enclosed

        results = {}
        if EnclosedAreasCalculator.ENLOSED_AREAS_COUNT in metrics:
            results[EnclosedAreasCalculator.ENLOSED_AREAS_COUNT] = Metric(
                name=EnclosedAreasCalculator.ENLOSED_AREAS_COUNT,
                value=best_enclosed,
                uncertainty=0.0,
                unit="areas"
            )
        return results

    def visualize_enclosed_areas(self, output_path: Optional[str] = None) -> np.ndarray:
        """Visualize the enclosed free areas on the map.
        
        The visualization shows the binary map with contours outlining the enclosed areas.
        
        Args:
            output_path: Optional path to save the visualization.
            
        Returns:
            Visualization image with enclosed areas marked.
        """
        # First, assign undefined areas to maximum occupancy.
        temp_map = self.map_data.copy()
        undefined_mask = (self.map_data == self.undefined_value)
        temp_map[undefined_mask] = self.occupied_value

        ret, binary = cv2.threshold(
            temp_map,
            0, 255,
            cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )
        # Find contours using Suzuki's algorithm (the underlying method for findContours).
        contours, hierarchy = cv2.findContours(
            binary, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE
        )
        vis_img = cv2.cvtColor(self.map_data, cv2.COLOR_GRAY2BGR)
        # Draw contours in red.
        cv2.drawContours(vis_img, contours, -1, (0, 0, 255), 1)

        if output_path:
            cv2.imwrite(output_path, vis_img)
        return vis_img


def main():
    """Command line interface for calculating the enclosed areas metric."""
    parser = argparse.ArgumentParser(
        description="Calculate the number of enclosed free areas in a PGM map."
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
        "--undefined-value",
        type=int,
        default=205,
        help="Pixel value for undefined areas (default: 205)"
    )
    parser.add_argument(
        "--free-value",
        type=int,
        default=0,
        help="Pixel value for free cells (default: 0)"
    )
    parser.add_argument(
        "--occupied-value",
        type=int,
        default=255,
        help="Pixel value for occupied cells (default: 255)"
    )
    parser.add_argument(
        "--candidate-start",
        type=int,
        default=255,
        help="Starting candidate occupancy value for undefined areas (default: 255)"
    )
    parser.add_argument(
        "--candidate-end",
        type=int,
        default=200,
        help="Ending candidate occupancy value (default: 200)"
    )
    parser.add_argument(
        "--candidate-step",
        type=int,
        default=5,
        help="Step size to decrease occupancy value (default: 5)"
    )
    parser.add_argument(
        "--visualize",
        action="store_true",
        help="Visualize the enclosed areas on the map"
    )
    parser.add_argument(
        "--debug",
        action="store_true",
        help="Show intermediate images during processing"
    )
    args = parser.parse_args()

    try:
        calculator = EnclosedAreasCalculator(
            map_path=args.pgm_file,
            yaml_path=args.yaml_file,
            undefined_value=args.undefined_value,
            free_value=args.free_value,
            occupied_value=args.occupied_value,
            candidate_start=args.candidate_start,
            candidate_end=args.candidate_end,
            candidate_step=args.candidate_step,
            debug=args.debug
        )

        metrics = calculator.calculate()
        print(metrics)

        if args.visualize:
            vis_img = calculator.visualize_enclosed_areas()
            show_image(vis_img, "Enclosed Areas")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1

if __name__ == "__main__":
    sys.exit(main())
