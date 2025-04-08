import argparse
import sys

from cartographer_tuner.metrics.calculators.pgm.occupied_proportion_calculator import OccupiedProportionCalculator

def main():
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
        calculator = OccupiedProportionCalculator(
            map_path=args.pgm_file,
            yaml_path=args.yaml_file,
            debug=args.debug
        )
        
        metrics = calculator.calculate()

        print(metrics)
        
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()
