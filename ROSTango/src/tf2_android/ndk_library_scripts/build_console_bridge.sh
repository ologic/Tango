#!/bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 console_bridge_source_dir"
    echo "  example: $0 /home/user/my_workspace/console_bridge"
    exit 1
fi

cmake_build $1
