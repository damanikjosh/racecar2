#!/bin/bash
set -e
source /ros2_ws/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/disable_fastdds_shm.xml
exec "$@"
