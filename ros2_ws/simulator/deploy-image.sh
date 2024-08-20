#!/usr/bin/env bash

# REMOTE_IP=143.248.96.55
REMOTE_IP=15.164.219.19
REMOTE_PORT=2220
REMOTE_USER=joshua
NO_BUILD=false
NO_DEPLOY=false

# Check for --no-build flag
for arg in "$@"
do
    if [ "$arg" == "--no-build" ]; then
        NO_BUILD=true
    fi
done

# Check for --no-deploy flag
for arg in "$@"
do
    if [ "$arg" == "--no-deploy" ]; then
        NO_DEPLOY=true
    fi
done

# Sync files
cd ../../
# rsync -a ros2_ws/ ${REMOTE_USER}@${REMOTE_IP}:~/racecar2/ros2_ws/
rsync -e "ssh -p ${REMOTE_PORT}" -a ros2_ws/ ${REMOTE_USER}@${REMOTE_IP}:~/racecar2/ros2_ws/
cd ros2_ws/simulator

if [ "$NO_BUILD" == false ]; then
    ssh -p ${REMOTE_PORT} ${REMOTE_USER}@${REMOTE_IP} "cd ~/racecar2/ros2_ws/simulator && docker compose build"
fi


# Only build if the --no-build or --no-deploy flags are not set
if [ "$NO_DEPLOY" == false ]; then

    ssh -p ${REMOTE_PORT} ${REMOTE_USER}@${REMOTE_IP} "cd ~/racecar2/ros2_ws/simulator && docker compose up"
    ssh -p ${REMOTE_PORT} ${REMOTE_USER}@${REMOTE_IP} "cd ~/racecar2/ros2_ws/simulator && docker compose down"
fi