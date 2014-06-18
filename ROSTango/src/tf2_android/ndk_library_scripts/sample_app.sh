#!/bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"
source $my_loc/config.sh
source $my_loc/utils.sh

if [ $# != 2 ]; then # || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 app_name tf2_ndk_path"
    echo "  example: $0 rba_sample_app /path/to/tf2_ndk"
    exit 1
fi

cmd_exists android || die 'could not find android executable in PATH'

[ "$ANDROID_NDK" = "" ] && die 'environment variable ANDROID_NDK not set'

rbndk=$(cd $2 && pwd)

lib_name=$1
package_name=com.ros.$(echo $1 | sed 's/_//g' | tr '[A-Z]' '[a-z]')
ndk_path=$ANDROID_NDK/ndk-build
module_path=$(dirname $rbndk)
tf2_ndk=$(basename $rbndk)

mkdir -p $lib_name/jni/src
prefix=$(cd $lib_name && pwd)

android create project -n $lib_name -t 1 -p $prefix -k $package_name -a $lib_name

subst_list="lib_name package_name ndk_path module_path tf2_ndk"
for s in $subst_list; do
    sedcmd="$sedcmd -e s,:{$s},\$$s,"
done
sedcmd="sed -e "$(eval echo $sedcmd)

subst() {
    target=$(echo $1 | sed 's,.in$,,')
    $sedcmd $my_loc/files/sample_app/$1 > $2/$target
}

files=$(cd $my_loc/files/sample_app && find . -name \*.in)
for f in $files; do
    subst $f $prefix
done

rest=$(cd $my_loc/files/sample_app && find . -type f | grep -v '.in$')
for f in $rest; do
    cp $my_loc/files/sample_app/$f $prefix/$f
done

# Copy app icon
mkdir -p $prefix/res/drawable
cp $my_loc/files/sample_app/ic_launcher.png $prefix/res/drawable/ic_launcher.png