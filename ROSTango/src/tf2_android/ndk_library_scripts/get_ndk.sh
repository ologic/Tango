#!/bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 prefix_path"
    echo "  example: $0 /home/user/my_workspace"
    exit 1
fi

prefix=$(cd $1 && pwd)
URL=http://dl.google.com/android/ndk/android-ndk-r9-$system.tar.bz2

download_bz2 $URL $prefix

echo 'done.'
echo 'please run the following:'
echo "  export ANDROID_NDK=$prefix/android-ndk-r9"
