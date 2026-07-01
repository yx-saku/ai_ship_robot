#!/bin/bash

DEVCONTAINER=$(cd $(dirname $0); pwd)

mkdir -p \
    "${DEVCONTAINER}/.config/kilo" \
    "${DEVCONTAINER}/.local/state/kilo" \
    "${DEVCONTAINER}/.local/share/kilo" \
    "${DEVCONTAINER}/.local/ros/rosdep" \
    "${DEVCONTAINER}/.local/ros/log" \
    "${DEVCONTAINER}/.local/gui"