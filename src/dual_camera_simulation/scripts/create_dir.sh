#!/bin/bash

# This script creates a directory if it doesn't exist
# Usage: ./create_dir.sh /path/to/directory

if [ $# -ne 1 ]; then
    echo "Usage: $0 <directory_path>"
    exit 1
fi

DIRECTORY=$1

if [ ! -d "$DIRECTORY" ]; then
    echo "Creating directory: $DIRECTORY"
    mkdir -p "$DIRECTORY"
    if [ $? -ne 0 ]; then
        echo "Error: Failed to create directory: $DIRECTORY"
        exit 1
    fi
else
    echo "Directory already exists: $DIRECTORY"
fi

exit 0