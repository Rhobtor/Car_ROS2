#!/usr/bin/env bash
# Prepara el entorno y ejecuta el comando del contenedor
set -e
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
exec "$@"