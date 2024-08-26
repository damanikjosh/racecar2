#!/usr/bin/env bash

REMOTE_USER=lics
REMOTE_IP=192.168.24.18
REMOTE_PORT=22
SERVER_IP=192.168.24.2
NO_DEPLOY=false

# Check for --no-deploy flag
for arg in "$@"
do
    if [ "$arg" == "--no-deploy" ]; then
        NO_DEPLOY=true
    fi
done

docker push localhost:5001/racecar_simulator:latest
ssh $REMOTE_USER@$REMOTE_IP "docker pull $SERVER_IP:5001/racecar_simulator:latest && docker tag $SERVER_IP:5001/racecar_simulator:latest racecar_simulator:latest"
cd ../../
rsync -e "ssh -p ${REMOTE_PORT}" -a ros2_ws/ ${REMOTE_USER}@${REMOTE_IP}:~/racecar2/ros2_ws/

if [ "$NO_DEPLOY" == false ]; then
    ssh $REMOTE_USER@$REMOTE_IP "cd ~/racecar2/ros2_ws/simulator && docker compose up"
    ssh $REMOTE_USER@$REMOTE_IP "cd ~/racecar2/ros2_ws/simulator && docker compose kill"
fi