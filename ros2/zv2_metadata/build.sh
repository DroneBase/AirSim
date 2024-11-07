#!/usr/bin/env bash

SCRIPT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")

IMAGE_VERSION=latest
IMAGE_NAME=airsim-ros-zv2-metadata
IMAGE_NAME_FULL=${IMAGE_NAME}:${IMAGE_VERSION}

function docker_build() {
    docker build \
        --progress plain \
        -f ${SCRIPT_DIR}/Dockerfile \
	-t ${IMAGE_NAME_FULL} \
        ${SCRIPT_DIR}/
}

# Main
docker_build

