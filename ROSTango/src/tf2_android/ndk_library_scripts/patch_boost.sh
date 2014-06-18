#!/bin/bash

my_loc="$(cd "$(dirname $0)" && pwd)"

if [ $# != 1 ] || [ $1 == '-h' ] || [ $1 == '--help' ]; then
    echo "Usage: $0 boost_source_dir"
    echo "  example: $0 /home/user/my_workspace/boost_1_47_0"
    exit 1
fi

pdir="$my_loc/files/boost"

if [ ! -d $pdir ]; then
    echo 'no patches found for boost'
    exit 1
fi

cd $1 && cat $pdir/*.patch | patch -Np1 --no-backup-if-mismatch

exit 0
