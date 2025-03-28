import argparse

import cv2
import sys

from cartographer_tuner.metrics.calculators.pgm.corner_count_calculator import CornerCountCalculator
from cartographer_tuner.utils.visualization import show_image

def main():
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
    parser.add_argument(
        "--max-corners",
        type=int,
        default=10000,
        help="Maximum number of corners to detect (default: 10000)"
    )
    args = parser.parse_args()
    
    try:
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
            max_corners=args.max_corners,
            debug=args.debug
        )
        
        metrics = calculator.calculate()
        
        print(metrics)

        if args.visualize:
            vis_img = cv2.cvtColor(calculator.map_data, cv2.COLOR_GRAY2BGR)
            for y, x in calculator._corners:
                cv2.circle(vis_img, (x, y), 3, (0, 0, 255), -1)
            show_image(vis_img, "Detected Corners")
        
    
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
