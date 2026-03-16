#!/bin/bash
set -e

docker run -it --rm \
    --net=host \
    nimbro_topic_transport_ros2:latest \
    "$@"
