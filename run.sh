#!/bin/bash

set -e

if [ -z "$1" ]; then
    echo "Usage: $0 <executable_name> [arguments]"
    exit 1
fi

EXECUTABLE_NAME=$1
shift # Remove the executable name from arguments
EXECUTABLE_PATH="./build/$EXECUTABLE_NAME/$EXECUTABLE_NAME"

if [ ! -f "$EXECUTABLE_PATH" ]; then
    echo "Error: Executable '$EXECUTABLE_NAME' not found in './build/$EXECUTABLE_NAME/'"
    exit 1
fi

echo "Running $EXECUTABLE_NAME..."
$EXECUTABLE_PATH "$@"