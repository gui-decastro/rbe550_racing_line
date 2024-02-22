#!/bin/bash
# Script that will build the docker image based on the content of the Dockerfile
# Run this if an image has not been built in your machine, or if change have been made to the Dockerfile

# Load docker config
source docker_config.sh

# Build docker image
docker build -t $IMAGE_NAME .