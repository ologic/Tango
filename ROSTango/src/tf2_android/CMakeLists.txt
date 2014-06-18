cmake_minimum_required(VERSION 2.8.3)
project(tf2_android)

find_package(catkin REQUIRED rosjava_build_tools)

catkin_android_setup(assembleRelease publishToMavenLocal)

catkin_package()

