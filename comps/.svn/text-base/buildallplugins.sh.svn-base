#!/bin/sh
set -e

unset ROS_ROOT

for pkg in generalik cbirrt2 manipulation2
do
   cd planning/$pkg
   make clean
   make install
   cd -
done

cd planning/herb2ikfast
rm -rf CMakeCache.txt CMakeFiles
cmake .
make install
cd -
