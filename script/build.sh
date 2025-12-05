#!/bin/bash


set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

echo "Building ROS2 project in: $PROJECT_DIR"

cd "$PROJECT_DIR"

echo "Building ROS2 package..."
colcon build

echo "ROS2 build completed successfully!"

