#!/bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 prefix_path"
    echo "  example: $0 /home/user/my_workspace"
    exit 1
fi

if [ ! -d $1 ]; then
    mkdir -p $1
fi

prefix=$(cd $1 && pwd)

run_cmd() {
    cmd=$1.sh
    shift
    $my_loc/$cmd $@ || die "$cmd $@ died with error code $?"
}

if [ -z $ANDROID_NDK ] ; then
    [ -d $prefix/android-ndk-r9 ] || run_cmd get_ndk $prefix
    export ANDROID_NDK=$prefix/android-ndk-r9
fi

#[ -d $standalone_toolchain_path ] || run_cmd setup_standalone_toolchain

mkdir -p $prefix/libs

if [ ! -d $prefix/libs/boost_1_47_0 ]; then
    run_cmd get_boost $prefix/libs
    run_cmd patch_boost $prefix/libs/boost_1_47_0
    run_cmd prepare_boost $prefix/libs/boost_1_47_0
fi

[ -d $prefix/libs/catkin ] || run_cmd get_catkin $prefix/libs
[ -d $prefix/libs/console_bridge ] || run_cmd get_console_bridge $prefix/libs

[ -d $prefix/target ] || mkdir -p $prefix/target
export CMAKE_PREFIX_PATH=$prefix/target

[ -e $prefix/android.toolchain.cmake ] || ( cd $prefix && download 'https://raw.github.com/taka-no-me/android-cmake/master/android.toolchain.cmake' && cat $my_loc/files/android.toolchain.cmake.addendum >> $prefix/android.toolchain.cmake)
export RBA_TOOLCHAIN=$prefix/android.toolchain.cmake

run_cmd build_catkin $prefix/libs/catkin
. $prefix/target/setup.bash
run_cmd get_ros_stuff $prefix/libs

run_cmd build_boost $prefix/libs/boost_1_47_0
run_cmd build_console_bridge $prefix/libs/console_bridge
run_cmd build_tf2

run_cmd setup_ndk_project $prefix/tf2_ndk
( cd $prefix && run_cmd sample_app sample_app $prefix/tf2_ndk )

echo
echo 'done.'
echo 'summary of what just happened:'
echo '  target/      was used to build static libraries for ros software'
echo '    include/   contains headers'
echo '    lib/       contains static libraries'
echo '  tf2_ndk/     is a NDK sub-project that can be imported into an NDK app'
echo '  sample_app/  is an example of such an app, a native activity that uses tf2'
echo
echo 'you might now cd into sample_app/, run "ant debug install", and if an'
echo 'android emulator is running, the app will be flashed onto it.'
echo
