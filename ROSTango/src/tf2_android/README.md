tf2_android
===========

Android NDK Wrapper of tf2

How to use tf2_android:

Install ROSJava using these instructions:
http://wiki.ros.org/android/Tutorials/hydro/Installation%20-%20Ros%20Development%20Environment

Now chain the tf2 workspace to the android workspace:

	mkdir -p ~/tf2a/src
	cd ~/tf2a/src
	git clone [this_repo]
	cd ..
	source ~/android/devel/setup.bash
	catkin_make

Now make your own workspace:
	mkdir -p ~/my_app_wksp/src
	cd ~/my_app_wksp/src

(Run catkin_create_android_project and catkin_create_android_pkg):
http://wiki.ros.org/rosjava_build_tools

	cd ..
	catkin_make

To use tf2_ros as a dependency, edit the build.gradle of your android_pkg and update
depdencies to be the following:

dependencies {
		compile 'org.ros.tf2:tf2_ros:0.0.0-SNAPSHOT'
}

Now you should be able to use the tf2 rosjava classes in your app.

Updating the NDK Lib
--------------------

Advanced users only:

If for some reason you need to update the NDK lib or change the JNI interface,
these are instructions for doing so.  YOU DO NOT NEED TO DO THIS TO USE TF2
IN ROSJAVA.  A precompiled ndk_library is already included in tf2_ros.

In some non-catkin folder, checkout the ndk_library_scripts version of this repo.
Follow all of its directions and make sure that the sample app starts on your device
or emulator.

Now, back in your rosjava tf2 workspace, copy the 'tf2_ndk' folder from your
scripts workspace to '[this_repo]/tf2_ros/src/main/jni/tf2_ndk.'

To rebuild the library:

	cd [my_workspace]
	cd src/tf2_android/tf2_ros/src/main
	export NDK_MODULE_PATH=$PWD/jni
	ndk-build

This regenerated libs/armeabi/libtf2_ros.so for you using the jni/src files.  Note that if you need to regenerate the org_ros_tf2_ros_Buffer.h file:

	cd [my_workspace]
	cd src/tf2_android/tf2_ros/build/classes/release
	javah org.ros.tf2_ros.Buffer
	cp org_ros_tf2_ros_Buffer.h ../../../src/main/jni/src/org_ros_tf2_ros_Buffer.h

