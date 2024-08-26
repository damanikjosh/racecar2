#!/usr/bin/env bash

# Build the Docker image
cd ..
docker build -t racecar_simulator:latest -f simulator/Dockerfile .
docker tag racecar_simulator:latest localhost:5001/racecar_simulator:latest

# SSH into the robot and pull the image