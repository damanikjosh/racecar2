REMOTE_HOST=lics@192.168.24.102
IMAGE_NAME=racecar_robot
TAG=latest
CONTAINER_NAME=ros
REGISTRY=macbook:5001
REGISTRY_CONTAINER_NAME=registry

all: build push run

build:
	cd ../ && docker build -t $(REGISTRY)/$(IMAGE_NAME):$(TAG) -f robot/Dockerfile .

push:
	if [ -z "$(shell docker ps -q -f name=$(REGISTRY_CONTAINER_NAME))" ]; then \
			if [ -n "$(shell docker ps -aq -f name=$(REGISTRY_CONTAINER_NAME))" ]; then \
					docker start $(REGISTRY_CONTAINER_NAME); \
			else \
					docker run -d -p 5001:5000 --name $(REGISTRY_CONTAINER_NAME) registry:2; \
			fi \
	fi
	docker push $(REGISTRY)/$(IMAGE_NAME):$(TAG)
	ssh $(REMOTE_HOST) "docker pull $(REGISTRY)/$(IMAGE_NAME):$(TAG)"
	ssh $(REMOTE_HOST) "docker tag $(REGISTRY)/$(IMAGE_NAME):$(TAG) $(IMAGE_NAME):$(TAG)"
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