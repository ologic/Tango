#!/bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh


if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 boost_source_dir"
    echo "  example: $0 /home/user/my_workspace/boost_1_47_0"
    exit 1
fi

boost_libs="date_time,signals,system,thread"

prefix=$(cd $1 && pwd)

cd $1
sh bootstrap.sh --with-libraries=$boost_libs
./b2 toolset=gcc-android4.6 link=static runtime-link=static target-os=linux --stagedir=android

[ "$CMAKE_PREFIX_PATH" = "" ] && die 'could not find target basedir. Have you run build_catkin.sh and sourced setup.bash?'
mkdir -p $CMAKE_PREFIX_PATH/lib
cd $CMAKE_PREFIX_PATH/lib
ln -sf $prefix/android/lib/lib*.a ./
mkdir -p ../include && cd ../include
ln -sf $prefix/boost ./
