#!/bin/bash

# Function to display usage information
usage() {
  echo "Usage: $0 -t <DATE> -r <RUN> -p <DATA_PATH> <DATA_TYPE>"
  echo "Example: $0 -t 2011_09_26 -r 0002 -p /path/to/data raw_synced"
  exit 1
}

# Parse input arguments
while getopts ":t:r:p:" opt; do
  case $opt in
    t) DATE="$OPTARG" ;;  # Date argument
    r) RUN="$OPTARG" ;;   # Run argument
    p) DATA_PATH="$OPTARG" ;;  # Custom data path
    *) 
      echo "Invalid option: -$OPTARG"
      usage
      ;;
  esac
done
shift $((OPTIND - 1))

# Remaining argument is the data type
if [ -z "$1" ]; then
  echo "You must specify a data type (e.g., raw_synced)."
  usage
fi
DATA_TYPE=$1

# Validate required arguments
if [ -z "$DATE" ] || [ -z "$RUN" ] || [ -z "$DATA_PATH" ]; then
  echo "Error: -t <DATE>, -r <RUN>, and -p <DATA_PATH> are all required."
  usage
fi

# Construct and execute the Docker run command
docker run -v "$DATA_PATH:/data" -it kitti2bag -t "$DATE" -r "$RUN" "$DATA_TYPE"
