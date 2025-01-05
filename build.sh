#!/bin/bash

set -e

CLEAN_BUILD=false

# Parse flags
while getopts "c" opt; do
    case $opt in
        c)
            CLEAN_BUILD=true
            ;;
        *)
            echo "Usage: $0 [-c]"
            echo "  -c  Perform a clean build"
            exit 1
            ;;
    esac
done

if [ "$CLEAN_BUILD" = true ]; then
    echo "Performing a clean build..."
    rm -rf build
    mkdir build
else
    if [ ! -d "build" ]; then
        echo "Creating build directory..."
        mkdir build
    fi
fi

echo "Configuring project with CMake..."
cd build
cmake .. # Only reconfigure if necessary

echo "Building project..."
make -j$(nproc) # Incremental build

echo "Build completed successfully!"