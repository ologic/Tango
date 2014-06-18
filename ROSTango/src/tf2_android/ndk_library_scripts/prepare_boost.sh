#!/bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 boost_source_dir"
    echo "  example: $0 /home/user/my_workspace/boost_1_47_0"
    exit 1
fi

[ "$ANDROID_NDK" = "" ] && die 'environment variable ANDROID_NDK not set'

boost_path=$(cd $1 && pwd)
ndk_path=$ANDROID_NDK

cat << EOF > $boost_path/tools/build/v2/user-config.jam
buildPlatform = $system ;
androidAPI = $platform ;
gccVersion = $gcc_version ;
ANDROID_NDK = $ndk_path ;
EOF

cat $my_loc/files/boost/user-config.jam.partial >> $boost_path/tools/build/v2/user-config.jam
