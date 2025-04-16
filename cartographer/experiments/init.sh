#!/bin/bash

set -e

if [ $# -eq 0 ]; then
    echo "Usage: $0 <experiment_name>"
    exit 1
fi

PROJECT_ROOT="$(dirname "$(dirname "$(realpath "$0")")")"
EXPERIMENT_NAME=$1
DATE_TIME=$(date "+%Y-%m-%d_%H-%M-%S")
EXPERIMENT_DIR="${PROJECT_ROOT}/experiments/${DATE_TIME}_${EXPERIMENT_NAME}"
EXPERIMENT_ID="${DATE_TIME}_${EXPERIMENT_NAME}"

mkdir -p "${EXPERIMENT_DIR}"

if ! git rev-parse --is-inside-work-tree > /dev/null 2>&1; then
    echo "ERROR: This script must be run inside a Git repository."
    echo "For reproducibility, all experiments must be version controlled."
    exit 1
fi

# Check for uncommitted changes
if [[ -n $(git status -s) ]]; then
    echo "ERROR: There are uncommitted changes in the repository."
    echo "For full reproducibility, commit all changes before running experiments."
    exit 1
fi

# Get Git information
GIT_COMMIT=$(git rev-parse HEAD)
GIT_REPO_URL=$(git config --get remote.origin.url)
GIT_BRANCH=$(git rev-parse --abbrev-ref HEAD)

# Save Git info to metadata
cat > "${EXPERIMENT_DIR}/metadata.json" << EOF
{
    "experiment_id": "${EXPERIMENT_ID}",
    "created_at": "$(date -u +"%Y-%m-%dT%H:%M:%SZ")",
    "git": {
        "commit": "${GIT_COMMIT}",
        "repository": "${GIT_REPO_URL}",
        "branch": "${GIT_BRANCH}"
    }
}
EOF

# System information
cat > "${EXPERIMENT_DIR}/system_info.txt" << EOF
Date: $(date)
Hostname: $(hostname)
OS: $(uname -a)
EOF

# Create a runner script template
cat > "${EXPERIMENT_DIR}/run_experiment.sh" << EOF
#!/bin/bash

# This script must be run inside docker container. 
# It contains the sequence of commands that must be executed to run the experiment.

EOF

chmod +x "${EXPERIMENT_DIR}/run_experiment.sh"

# Create a README template
cat > "${EXPERIMENT_DIR}/README.md" << EOF
# Experiment: ${EXPERIMENT_NAME}

COMMIT: ${GIT_COMMIT}

## Overview

## Results

EOF

echo "Experiment template created at: ${EXPERIMENT_DIR}"