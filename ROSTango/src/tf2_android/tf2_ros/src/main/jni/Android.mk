LOCAL_PATH := $(call my-dir)

$(info $(LOCAL_PATH))

include $(CLEAR_VARS)

LOCAL_MODULE    := tf2_ros
LOCAL_SRC_FILES := src/org_ros_tf2_ros_Buffer.cpp
LOCAL_C_INCLUDES := $(LOCAL_PATH)/include
LOCAL_LDLIBS := -landroid
LOCAL_STATIC_LIBRARIES := android_native_app_glue tf2a

include $(BUILD_SHARED_LIBRARY)

$(info $(LOCAL_PATH))

$(call import-module,android/native_app_glue)
$(call import-module,tf2_ndk)