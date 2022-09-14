#!/bin/bash
set -e

# setup ros environment
source "/opt/ros/foxy/setup.bash"
exec "$@"