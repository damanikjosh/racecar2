REMOTE_HOST=lics@192.168.24.18
IMAGE_NAME=racecar_robot
# Set LOCAL_BUILDER to true if you want to build the image locally
LOCAL_BUILDER=false
TAG=latest
CONTAINER_NAME=ros
REGISTRY=macbook:5001
REGISTRY_CONTAINER_NAME=registry

all: build push run

build:
	if [ "$(LOCAL_BUILDER)" = "true" ]; then \
		cd ../ && docker build -t $(REGISTRY)/$(IMAGE_NAME):$(TAG) -f robot/Dockerfile .; \
	else \
		rsync -avz ../ $(REMOTE_HOST):~/racecar2/ros2_ws/; \
		ssh $(REMOTE_HOST) "cd ~/racecar2/ros2_ws && docker build -t $(REGISTRY)/$(IMAGE_NAME):$(TAG) -f robot/Dockerfile ."; \
	fi

push:
	if [ "$(LOCAL_BUILDER)" = "true" ]; then \
		docker push $(REGISTRY)/$(IMAGE_NAME):$(TAG); \
		ssh $(REMOTE_HOST) "cd ~/racecar2/ros2_ws/robot && docker pull $(REGISTRY)/$(IMAGE_NAME):$(TAG)"; \
		ssh $(REMOTE_HOST) "cd ~/racecar2/ros2_ws/robot && docker tag $(REGISTRY)/$(IMAGE_NAME):$(TAG) $(IMAGE_NAME):$(TAG)"; \
	fi
	rsync -avz ../ $(REMOTE_HOST):~/racecar2/ros2_ws/

run:
	ssh $(REMOTE_HOST) "cd ~/racecar2/ros2_ws/robot && docker compose kill && docker compose up -d"
	ssh -t $(REMOTE_HOST) "docker exec -it $(CONTAINER_NAME) bash"
	ssh $(REMOTE_HOST) "cd ~/racecar2/ros2_ws/robot && docker compose kill"

attach:
	ssh -t $(REMOTE_HOST) "docker exec -it $(CONTAINER_NAME) bash"

ssh:
	ssh -t $(REMOTE_HOST) bash

download:
	rsync -avz $(REMOTE_HOST):~/racecar2/ros2_ws/maps/ ../maps/

kill:
	ssh $(REMOTE_HOST) "cd ~/racecar2/ros2_ws/robot && docker compose kill"