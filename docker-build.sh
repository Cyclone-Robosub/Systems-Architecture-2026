#!/bin/bash
# shellcheck disable=SC2034,SC1091

# Check if the first argument is "skip-wsl" and if the script is running in WSL
# This is used by the Dev Container to avoid starting the container via this script when running on Windows
if [[ "$1" == "skip-wsl" && -f /proc/sys/fs/binfmt_misc/WSLInterop ]]; then
    echo "'skip-wsl' is specified and the script is running in WSL. Exiting..."
    exit 0
fi

