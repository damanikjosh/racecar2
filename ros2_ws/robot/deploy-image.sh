#!/usr/bin/env bash

ROBOT_USER=lics
ROBOT_IP=192.168.55.1
SERVER_IP=192.168.55.100

docker push localhost:5001/racecar_robot:latest
ssh $ROBOT_USER@$ROBOT_IP "docker pull $SERVER_IP:5001/racecar_robot:latest && docker tag $SERVER_IP:5001/racecar_robot:latest racecar_robot:latest"
cd ..
scp -r robot $ROBOT_USER@$ROBOT_IP:~/racecar2/ros2_ws
cd robot