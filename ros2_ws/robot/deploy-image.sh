#!/usr/bin/env bash

ROBOT_USER=lics
ROBOT_IP=192.168.24.21
ROBOT_PORT=22
SERVER_IP=192.168.24.2
NO_DEPLOY=false

# Check for --no-deploy flag
for arg in "$@"
do
    if [ "$arg" == "--no-deploy" ]; then
        NO_DEPLOY=true
    fi
done

docker push localhost:5001/racecar_robot:latest
ssh $ROBOT_USER@$ROBOT_IP "docker pull $SERVER_IP:5001/racecar_robot:latest && docker tag $SERVER_IP:5001/racecar_robot:latest racecar_robot:latest"
cd ../../
rsync -e "ssh -p ${ROBOT_PORT}" -a ros2_ws/ ${ROBOT_USER}@${ROBOT_IP}:~/racecar2/ros2_ws/

if [ "$NO_DEPLOY" == false ]; then
    ssh $ROBOT_USER@$ROBOT_IP "cd ~/racecar2/ros2_ws/robot && docker compose up"
    ssh $ROBOT_USER@$ROBOT_IP "cd ~/racecar2/ros2_ws/robot && docker compose down"
fi