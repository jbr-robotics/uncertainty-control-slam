#!/bin/bash

if [ $# -eq 0 ]; then
    echo "Usage: $0 <experiment_name>"
    exit 1
fi

PROJECT_ROOT="$(dirname "${PWD}")"
EXPERIMENT_NAME=$1
EXPERIMENT_DIR="${PROJECT_ROOT}/experiments/${EXPERIMENT_NAME}"
DATE_TIME=$(date "+%Y-%m-%d_%H-%M-%S")
EXPERIMENT_ID="${DATE_TIME}_${EXPERIMENT_NAME}"
FULL_EXPERIMENT_DIR="${EXPERIMENT_DIR}/${EXPERIMENT_ID}"

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

mkdir -p "${FULL_EXPERIMENT_DIR}/"{config,data,results,src,logs}

# Get Git information
GIT_COMMIT=$(git rev-parse HEAD)
GIT_REPO_URL=$(git config --get remote.origin.url)
GIT_BRANCH=$(git rev-parse --abbrev-ref HEAD)

# Save Git info to metadata
cat > "${FULL_EXPERIMENT_DIR}/metadata.json" << EOF
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
cat > "${FULL_EXPERIMENT_DIR}/system_info.txt" << EOF
Date: $(date)
Hostname: $(hostname)
OS: $(uname -a)
EOF

# Create a runner script template
cat > "${FULL_EXPERIMENT_DIR}/run_experiment.sh" << EOF
#!/bin/bash

# This script must be run inside docker container. 

# Run experiment script for ${EXPERIMENT_ID}
echo "Running experiment: ${EXPERIMENT_ID}"


# Run the experiment

echo "Experiment completed"
EOF

chmod +x "${FULL_EXPERIMENT_DIR}/run_experiment.sh"

# Create a README template
cat > "${FULL_EXPERIMENT_DIR}/README.md" << EOF
# Experiment: ${EXPERIMENT_NAME}

COMMIT: ${GIT_COMMIT}

## Overview

## Results

EOF

echo "Experiment template created at: ${FULL_EXPERIMENT_DIR}"