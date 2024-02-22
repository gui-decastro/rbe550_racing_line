#!/bin/bash

PWD="$(pwd)"
IMAGE_NAME="racing_line_img"
CONTAINER_NAME="RACING_LINE"

# Get container id if it exists
CONTAINER_ID=`docker ps -aqf "name=^/${CONTAINER_NAME}$"`
echo "CONTAINER_ID: ${CONTAINER_ID}"

# Create new container if there are none running
if [ -z "${CONTAINER_ID}" ]; then
    echo -e "\e[36mCreating new RACING_LINE docker container.\e[0m"
    xhost +local:root
    docker run --rm \
        -it \
        --privileged \
        --network=host \
        -e DISPLAY=$DISPLAY \
        -v "${PWD}/catkin_ws:/catkin_ws" \
        --name="${CONTAINER_NAME}" \
        "${IMAGE_NAME}"
# if there is already a container running, just attach to it
else
    echo -e "\e[35mFound running ${CONTAINER_NAME} container, attaching bash...\e[0m"
    docker exec -it ${CONTAINER_ID} bash
fi