REMOTE_HOST=lics@192.168.24.4
IMAGE_NAME=racecar_simulator
LOCAL_BUILDER=true
TAG=latest
CONTAINER_NAME=ros
REGISTRY=macbook:5001
REGISTRY_CONTAINER_NAME=registry

all: build push run

# build:
# 	cd ../ && docker build -t $(REGISTRY)/$(IMAGE_NAME):$(TAG) -f simulator/Dockerfile .
build:
	if [ "$(LOCAL_BUILDER)" = "true" ]; then \
		cd ../ && docker build -t $(REGISTRY)/$(IMAGE_NAME):$(TAG) -f simulator/Dockerfile .; \
	else \
		rsync -avz ../ $(REMOTE_HOST):~/racecar2/ros2_ws/; \
		ssh $(REMOTE_HOST) "cd ~/racecar2/ros2_ws/simulator && docker build -t $(REGISTRY)/$(IMAGE_NAME):$(TAG) -f Dockerfile ."; \
	fi
	rsync -avz ../ $(REMOTE_HOST):~/racecar2/ros2_ws/
	ssh $(REMOTE_HOST) "cd ~/racecar2/ros2_ws/simulator && docker build -t $(IMAGE_NAME):$(TAG) -f Dockerfile ."

push:
	rsync -avz ../ $(REMOTE_HOST):~/racecar2/ros2_ws/


run:
	ssh $(REMOTE_HOST) "cd ~/racecar2/ros2_ws/simulator \
	  && docker compose kill \
		&& docker compose up -d"
	ssh -t $(REMOTE_HOST) "docker exec -it $(CONTAINER_NAME) bash"
	ssh $(REMOTE_HOST) "cd ~/racecar2/ros2_ws/simulator && docker compose kill"

attach:
	ssh -t $(REMOTE_HOST) "docker exec -it $(CONTAINER_NAME) bash"

ssh:
	ssh -t $(REMOTE_HOST) bash

download:
	rsync -avz $(REMOTE_HOST):~/racecar2/ros2_ws/maps/ ../maps/

kill:
	ssh $(REMOTE_HOST) "cd ~/racecar2/ros2_ws/simulator && docker compose kill"