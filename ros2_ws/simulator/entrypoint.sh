#!/bin/bash

set -e
source /ros2_ws/install/setup.bash
# sleep for 3 seconds to wait for the network to be ready
sleep 3
exec "$@"
