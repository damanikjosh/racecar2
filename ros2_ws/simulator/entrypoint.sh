#!/bin/bash

set -e
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///cyclonedds.xml
source /ros2_ws/install/setup.bash
# sleep 3
exec "$@"
