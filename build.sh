#!/bin/bash

set -e

echo "Cleaning build directory..."
rm -rf build
mkdir build

echo "Configuring project with CMake..."
cd build
cmake ..

echo "Building project..."
make -j$(nproc)

echo "Build completed successfully!"