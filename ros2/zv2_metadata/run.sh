#!/usr/bin/env bash

SCRIPT_DIR=$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")

IMAGE_VERSION=latest
IMAGE_NAME=airsim-ros-zv2-metadata
IMAGE_NAME_FULL=${IMAGE_NAME}:${IMAGE_VERSION}

docker run -it --rm --name airsim-ros-zv2-metadata \
    -v /${SCRIPT_DIR}/bags:/bags \
    -v /${SCRIPT_DIR}/images:/images \
    ${IMAGE_NAME_FULL} "${1}"

