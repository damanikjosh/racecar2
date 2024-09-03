#!/usr/bin/env bash

# Path to the directory containing docker-compose.yml
DOCKER_COMPOSE_DIR="/home/lics/racecar2/ros2_ws/robot"
DOCKER_CONTAINER_NAME="ros"

# Function to start the Docker Compose
start_docker_compose() {
    if docker ps | grep -q "$DOCKER_CONTAINER_NAME"; then
        echo "Docker containers are already running."
    else
        echo "Starting Docker containers..."
        cd "$DOCKER_COMPOSE_DIR"
        docker compose up -d
    fi
}

# Function to stop the Docker Compose
stop_docker_compose() {
    if docker ps | grep -q "$DOCKER_CONTAINER_NAME"; then
        echo "Stopping Docker containers..."
        cd "$DOCKER_COMPOSE_DIR"
        docker compose kill
    else
        echo "Docker containers are not running."
    fi
}

# Infinite loop to monitor the joystick connection
while true
do
    if [ -e /dev/input/js0 ]; then
        echo "Joystick connected."
        start_docker_compose
        sleep 5  # Check every 5 seconds
    else
        echo "Joystick disconnected."
        stop_docker_compose
        sleep 1  # Check every 1 second
    fi
done
