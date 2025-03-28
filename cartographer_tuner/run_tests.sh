#!/bin/bash

set -e

docker build -t cartographer_tuner_test -f Dockerfile --progress=plain .
docker run -it cartographer_tuner_test
