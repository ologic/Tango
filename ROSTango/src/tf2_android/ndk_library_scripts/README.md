These scripts will (hopefully) help you build static libraries
for tf2 for android and setup a sample application.

You will need android SDK installed and the 'android' program
location in the $PATH.

INSTALL
-------

The `do_everything.sh` script will call all the other scripts
sequentially, you just have to give it a prefix path:

    #./do_everything.sh /path/to/workspace
    ./do_everything.sh ../tf2_workspace

NOTE: YOU WILL HAVE TO RUN THIS COMMAND MULTIPLE TIMES DUE TO A
CMAKE ERROR WITH pthread WITH THE NDK TOOLCHAIN!!!!!!

You can also run each script individually, most of them have
a minimalistic help string to give you an idea of their parameters.

When finished, the script will give you a few lines of what it did.
If everything went fine, you will be able to do the following:

    #cd /path/to/workspace/sample_app
    cd ../tf2_workspace/sample_app
    ant debug

This will build the app. If you want to install it, run the following:

    ant debug install

This will install the app onto a virtual android device running in the
emulator.

To follow what the app does, you will need to read the log. The sdk has
a tool called `adb` located in `$SDK/platform-tools/`, you can follow the
log by running:

    $SDK/platform-tools/adb logcat

Good luck!

In order to use the compiled files in a JNI or NDK project, copy the 
tf2_ndk folder out of the tf2_workspace.  This project can be copied
into a jni folder and referenced using 'tf2a' library name.
