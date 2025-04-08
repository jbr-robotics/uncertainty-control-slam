import argparse

from cartographer_tuner.metrics.calculators.lua.pgm_based_calculator import LuaPgmMetricCalculator

def main():
    parser = argparse.ArgumentParser(
        description="Calculate PGM-based quality metrics for Cartographer maps"
    )
    
    parser.add_argument(
        "--bag_filename", 
        required=True,
        help="Path to input bag file"
    )
    parser.add_argument(
        "--config_dir", 
        required=True,
        help="Directory containing Lua configs"
    )
    parser.add_argument(
        "--config_basename", 
        required=True,
        help="Base name of config file"
    )
    parser.add_argument(
        "--skip_seconds", 
        type=int, 
        default=0,
        help="Seconds to skip from bag start"
    )
    parser.add_argument(
        "--resolution", 
        type=float, 
        default=0.05,
        help="Resolution of the map in meters per pixel"
    )
    parser.add_argument(
        "--tmp_dir",
        help="Directory for temporary files"
    )
    parser.add_argument(
        "--metrics", 
        nargs='+',
        choices=LuaPgmMetricCalculator.available_metrics(),
        default=None,
        help="Specific metrics to calculate (default: all)"
    )
    
    args = parser.parse_args()

    calculator = LuaPgmMetricCalculator(
        bag_filename=args.bag_filename,
        config_dir=args.config_dir,
        config_basename=args.config_basename,
        skip_seconds=args.skip_seconds,
        tmp_dir=args.tmp_dir,
        resolution=args.resolution,
    )
    
    try:
        results = calculator.calculate(args.metrics)
        
        for metric in results.values():
            print(metric)
            
    except Exception as e:
        print(f"Error calculating metrics: {e}")
    


if __name__ == "__main__":
    main()
