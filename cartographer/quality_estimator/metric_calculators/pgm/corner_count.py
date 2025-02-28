import os
import sys
import json
import argparse
import numpy as np
import cv2
from typing import Dict, List, Optional, Tuple

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


class CornerCountCalculator(BasePgmMetricCalculator):
    """Calculator for counting corners in PGM maps.
    
    This metric uses Gaussian-Laplace filtering and Harris corner detection
    to identify structural corners in the map, which can be an indicator
    of map quality.
    """
    
    # Define available metrics
    CORNER_COUNT = "corner_count"
    
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
        debug: bool = False
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
            debug: Whether to show intermediate images during processing
        """
        super().__init__(map_path, yaml_path)
        
        # Store parameters        
        # filter parameters
        self.filter_size = filter_size
        self.sigma = sigma
        # corner detection parameters
        self.block_size = block_size
        self.ksize = ksize
        self.k = k
        self.threshold = threshold
        self.min_distance = min_distance
        
        self.min_blob_size = min_blob_size
        
        # Debug flag
        self.debug = debug
        
        # Initialize results
        self._corners = None
    
    @staticmethod
    def get_available_metrics() -> List[str]:
        """Get list of available metrics for this calculator.
        
        Returns:
            List of metric names that can be calculated
        """
        return [
            CornerCountCalculator.CORNER_COUNT
        ]
    
    def _remap_values(self, map_data: np.ndarray) -> np.ndarray:
        """Remap the pixel values:
        Values <= 127 are mapped to 0 (empty or unknown)
        Values [128, 255] are linearly mapped to [1, 255] (probably occupied)
        """
        remapped = np.maximum(0, map_data - 127)
        remapped = remapped / np.max(remapped) * 255
        assert np.max(remapped) <= 255, "Kek"
        return remapped
    
    def _apply_gaussian_laplace(self, map_data: np.ndarray) -> np.ndarray:
        """Apply Gaussian-Laplace filtering to the map.
        
        This method applies Gaussian-Laplace filtering to the map to extract
        the abstracted structure of the map and removes small discrete dots.
        """
        blurred = cv2.GaussianBlur(
            map_data, 
            (self.filter_size, self.filter_size), 
            self.sigma
        )
        laplacian = cv2.Laplacian(blurred, cv2.CV_64F)
        return laplacian
    
    def detect_corners(self, map_data: np.ndarray) -> List[Tuple[int, int]]:
        if self.debug:
            show_image(map_data, "Original map")
        # Preprocess the map
        processed_map = self._remap_values(map_data)
        if self.debug:
            show_image(processed_map, "Remapped map")
        processed_map = self._apply_gaussian_laplace(processed_map)
        if self.debug:
            show_image(processed_map, "Gaussian-Laplace filtered map")
        
        # Use goodFeaturesToTrack with the Harris detector enabled to get distinct corners.
        corners = cv2.goodFeaturesToTrack(
            processed_map.astype(np.float32),
            maxCorners=1000,
            qualityLevel=self.threshold,
            minDistance=self.min_distance,
            blockSize=self.block_size,
            useHarrisDetector=True,
            k=self.k
        )
        
        # Convert corners from float to integer coordinates (y, x)
        if corners is not None:
            corners = np.intp(corners).reshape(-1, 2)
            self._corners = [(pt[1], pt[0]) for pt in corners]
        else:
            self._corners = []
        
        return self._corners
    
    def calculate(self, metrics: Optional[List[str]] = None) -> Dict[str, Metric]:
        """Calculate corner-based metrics.
        
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
        
        # Detect corners if not already done
        if self._corners is None:
            self._corners = self.detect_corners(self.map_data)
        
        # Calculate metrics
        results = {}
        
        if self.CORNER_COUNT in metrics:
            corner_count = len(self._corners)
            results[self.CORNER_COUNT] = Metric(
                name=self.CORNER_COUNT,
                value=corner_count,
                uncertainty=0.0,
                unit="corners"
            )
        
        return results
    
    def visualize_corners(self, output_path: Optional[str] = None) -> np.ndarray:
        """Visualize the detected corners on the map.
        
        Args:
            output_path: Optional path to save the visualization
            
        Returns:
            Visualization image with corners marked in red
        """
        vis_img = cv2.cvtColor(self.map_data, cv2.COLOR_GRAY2BGR)
        
        for y, x in self._corners:
            cv2.circle(vis_img, (x, y), 3, (0, 0, 255), -1)
        
        if output_path:
            cv2.imwrite(output_path, vis_img)
        
        return vis_img


def main():
    """Command line interface for calculating the corner count metric."""
    parser = argparse.ArgumentParser(
        description="Calculate the number of structural corners in a PGM map."
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
        "--block-size", 
        type=int, 
        default=2,
        help="Block size for Harris corner detection (default: 2)"
    )
    parser.add_argument(
        "--ksize", 
        type=int, 
        default=3,
        help="Aperture parameter for Sobel operator (default: 3)"
    )
    parser.add_argument(
        "--k", 
        type=float, 
        default=0.04,
        help="Harris detector free parameter (default: 0.04)"
    )
    parser.add_argument(
        "--threshold", 
        type=float, 
        default=0.01,
        help="Threshold for corner detection (default: 0.01)"
    )
    parser.add_argument(
        "--min-distance", 
        type=int, 
        default=10,
        help="Minimum distance between corners (default: 10)"
    )
    parser.add_argument(
        "--filter-size", 
        type=int, 
        default=5,
        help="Size of the Gaussian-Laplace filter (default: 5)"
    )
    parser.add_argument(
        "--sigma", 
        type=float, 
        default=1.0,
        help="Standard deviation for the Gaussian-Laplace filter (default: 1.0)"
    )
    parser.add_argument(
        "--min-blob-size", 
        type=int, 
        default=5,
        help="Minimum size of blobs to keep after filtering (default: 5)"
    )
    parser.add_argument(
        "--visualize", 
        action="store_true",
        help="Visualize the detected corners on the map"
    )
    parser.add_argument(
        "--debug", 
        action="store_true",
        help="Show intermediate images during processing"
    )
    
    args = parser.parse_args()
    
    try:
        # Create the metric calculator
        calculator = CornerCountCalculator(
            map_path=args.pgm_file,
            yaml_path=args.yaml_file,
            block_size=args.block_size,
            ksize=args.ksize,
            k=args.k,
            threshold=args.threshold,
            min_distance=args.min_distance,
            filter_size=args.filter_size,
            sigma=args.sigma,
            min_blob_size=args.min_blob_size,
            debug=args.debug
        )
        
        # Calculate the metric
        metrics = calculator.calculate()
        
        print(metrics)

        # Visualize the corners if requested
        if args.visualize:
            # Create visualization with corners marked in red
            vis_img = calculator.visualize_corners()
            show_image(vis_img, "Detected Corners")
        
    
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
