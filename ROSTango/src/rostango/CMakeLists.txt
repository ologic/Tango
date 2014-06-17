##############################################################################
# CMake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(rostango)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED rosjava_build_tools)
# Set the gradle targets you want catkin's make to run by default
catkin_android_setup(assembleRelease uploadArchives)
catkin_package()


##############################################################################
# Installation
##############################################################################

# If you are deploying android libraries (.aar's) uncomment this and
# change this to match the maven group name you have specified in the 
# allprojects closure the root build.gradle
#install(DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_MAVEN_DESTINATION}/com/github/rosjava/${PROJECT_NAME}/ 
#       DESTINATION ${CATKIN_GLOBAL_MAVEN_DESTINATION}/com/github/rosjava/${PROJECT_NAME})
