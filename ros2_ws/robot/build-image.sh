#!/usr/bin/env bash

# Build the Docker image
cd ..
docker build -t racecar_robot:latest -f robot/Dockerfile .
docker tag racecar_robot:latest localhost:5001/racecar_robot:latest

# SSH into the robot and pull the image