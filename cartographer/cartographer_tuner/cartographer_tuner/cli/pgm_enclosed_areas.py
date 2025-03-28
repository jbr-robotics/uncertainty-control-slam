import sys
import argparse
import cv2

from cartographer_tuner.metrics.calculators.pgm.enclosed_areas_calculator import EnclosedAreasCalculator
from cartographer_tuner.utils.visualization import show_image

def main():
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
            contours = calculator.enclosed_contours
            vis_img = cv2.cvtColor(calculator.map_data, cv2.COLOR_GRAY2BGR)
            cv2.drawContours(vis_img, contours, -1, (0, 0, 255), 1)
            show_image(vis_img, "Enclosed Areas")

    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        return 1

if __name__ == "__main__":
    sys.exit(main())
