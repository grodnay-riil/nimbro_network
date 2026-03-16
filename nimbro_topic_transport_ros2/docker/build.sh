#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"

docker build \
    -t nimbro_topic_transport_ros2:latest \
    -f "$SCRIPT_DIR/Dockerfile" \
    "$PKG_DIR"
