#include "org_ros_tf2_ros_Buffer.h"
#include "geometry_msgs/TransformStamped.h"
#include "tf2/buffer_core.h"
#include "tf2/exceptions.h"
#include <string>

#include <map>
#include <set>

boost::shared_ptr<tf2::BufferCore> buffer_core_(new tf2::BufferCore());

JNIEXPORT void JNICALL Java_org_ros_tf2_1ros_Buffer_newBufferWithCacheDuration
  (JNIEnv* env, jobject pThis, jint sec, jint nsec){
    ros::Duration d((uint32_t)sec, (uint32_t)nsec);
    buffer_core_.reset(new tf2::BufferCore(d));
  }

inline std::string stdStringFromjString(JNIEnv* env, jstring java_string){
    const char* tmp = env->GetStringUTFChars(java_string, NULL);
    std::string out(tmp);
    env->ReleaseStringUTFChars(java_string, tmp);
    return out;
}

JNIEXPORT jboolean JNICALL Java_org_ros_tf2_1ros_Buffer_setTransform
  (JNIEnv* env, jobject pThis, const jstring jframe_id, const jint sec, const jint nsec, const jstring jchild_frame_id,
   const jdouble tx, const jdouble ty, const jdouble tz,
   const jdouble rw, const jdouble rx, const jdouble ry, const jdouble rz, const jstring pAuthority, const jboolean pStatic){
    // Convert strings
    const std::string frame_id = stdStringFromjString(env, jframe_id);
    if(frame_id.empty()){
        return jboolean(false);
    }
    const std::string child_frame_id = stdStringFromjString(env, jchild_frame_id);
    if(child_frame_id.empty()){
        return jboolean(false);
    }
    const std::string authority = stdStringFromjString(env, pAuthority);
    if(authority.empty()){
        return jboolean(false);
    }

    // Convert to transformstamped
    geometry_msgs::TransformStamped tfs;
    tfs.header.frame_id = frame_id;
    tfs.header.stamp.sec = (uint32_t)sec;
    tfs.header.stamp.nsec = (uint32_t)nsec;
    tfs.child_frame_id = child_frame_id;
    tfs.transform.translation.x = (double)tx;
    tfs.transform.translation.y = (double)ty;
    tfs.transform.translation.z = (double)tz;
    tfs.transform.rotation.w = (double)rw;
    tfs.transform.rotation.x = (double)rx;
    tfs.transform.rotation.y = (double)ry;
    tfs.transform.rotation.z = (double)rz;


    // Insert into buffer
    bool static_tf = (bool)pStatic;
    return (jboolean)buffer_core_->setTransform(tfs, authority, static_tf);
  }

JNIEXPORT jdoubleArray JNICALL Java_org_ros_tf2_1ros_Buffer_lookupTransform__Ljava_lang_String_2Ljava_lang_String_2II
  (JNIEnv* env, jobject pThis, jstring pTargetFrame, jstring pSourceFrame, jint sec, jint nsec){

    // Convert strings
    //const std::string target_frame = stdStringFromjString(env, pTargetFrame);
    const char *target_frame = env->GetStringUTFChars(pTargetFrame, 0);
    if(target_frame == NULL){
        return env->NewDoubleArray( 0 );
    }
    //const std::string source_frame = stdStringFromjString(env, pSourceFrame);
    const char *source_frame = env->GetStringUTFChars(pSourceFrame, 0);
    if(source_frame == NULL){
        return env->NewDoubleArray( 0 );
    }

    // Convert time
    ros::Time lookup_time((uint32_t)sec, (uint32_t)nsec);
    geometry_msgs::TransformStamped tfs;
    try{
       tfs = buffer_core_->lookupTransform(target_frame, source_frame, lookup_time);
    } catch(const tf2::ConnectivityException& e){
      return env->NewDoubleArray( 0 ); ///< @TODO Should I rethrow a specific exception here?
    } catch(const tf2::ExtrapolationException& e){
      return env->NewDoubleArray( 0 );; ///< @TODO Should I rethrow a specific exception here?
    } catch(const tf2::LookupException& e){
      return env->NewDoubleArray( 0 ); ///< @TODO Should I rethrow a specific exception here?
    } catch (...) {
      return env->NewDoubleArray( 0 ); ///< @TODO Should I rethrow a specific exception here?
    }

    // Release strings
    env->ReleaseStringUTFChars(pTargetFrame, target_frame);
    env->ReleaseStringUTFChars(pSourceFrame, source_frame);


    // Encode results
    jdoubleArray dArray = env->NewDoubleArray( 9 );
    jdouble *arr = env->GetDoubleArrayElements(dArray, 0);
    
    arr[0] = tfs.transform.translation.x;
    arr[1] = tfs.transform.translation.y;
    arr[2] = tfs.transform.translation.z;
    arr[3] = tfs.transform.rotation.w;
    arr[4] = tfs.transform.rotation.x;
    arr[5] = tfs.transform.rotation.y;
    arr[6] = tfs.transform.rotation.z;
    arr[7] = tfs.header.stamp.sec;
    arr[8] = tfs.header.stamp.nsec;
    env->ReleaseDoubleArrayElements(dArray, arr, 0);

    return dArray;
  }

JNIEXPORT jdoubleArray JNICALL Java_org_ros_tf2_1ros_Buffer_lookupTransform__Ljava_lang_String_2IILjava_lang_String_2IILjava_lang_String_2
  (JNIEnv* env, jobject pThis, jstring pTargetFrame, jint pTargetSec, jint pTargetNSec,
   jstring pSourceFrame, jint pSourceSec, jint pSourceNSec, jstring pFixedFrame){
    // Convert strings
    const char *target_frame = env->GetStringUTFChars(pTargetFrame, 0);
    if(target_frame == NULL){
        return env->NewDoubleArray( 0 );
    }
    const char *source_frame = env->GetStringUTFChars(pSourceFrame, 0);
    if(source_frame == NULL){
        return env->NewDoubleArray( 0 );
    }
    const char *fixed_frame = env->GetStringUTFChars(pFixedFrame, 0);
    if(target_frame == NULL){
      return env->NewDoubleArray( 0 );
    }

    // Convert times
    ros::Time target_time((uint32_t)pTargetSec, (uint32_t)pTargetNSec);
    ros::Time source_time((uint32_t)pSourceSec, (uint32_t)pSourceNSec);

    geometry_msgs::TransformStamped tfs;
    try{
       tfs = buffer_core_->lookupTransform(target_frame, target_time, source_frame, source_time, fixed_frame);
    } catch(const tf2::ConnectivityException& e){
      return env->NewDoubleArray( 0 ); ///< @TODO Should I rethrow a specific exception here?
    } catch(const tf2::ExtrapolationException& e){
      return env->NewDoubleArray( 0 );; ///< @TODO Should I rethrow a specific exception here?
    } catch(const tf2::LookupException& e){
      return env->NewDoubleArray( 0 ); ///< @TODO Should I rethrow a specific exception here?
    } catch (...) {
      return env->NewDoubleArray( 0 ); ///< @TODO Should I rethrow a specific exception here?
    }

    // Release strings
    env->ReleaseStringUTFChars(pTargetFrame, target_frame);
    env->ReleaseStringUTFChars(pSourceFrame, source_frame);
    env->ReleaseStringUTFChars(pFixedFrame, fixed_frame);

    // Encode results
    jdoubleArray dArray = env->NewDoubleArray( 9 );
    jdouble *arr = env->GetDoubleArrayElements(dArray, 0);
    
    arr[0] = tfs.transform.translation.x;
    arr[1] = tfs.transform.translation.y;
    arr[2] = tfs.transform.translation.z;
    arr[3] = tfs.transform.rotation.w;
    arr[4] = tfs.transform.rotation.x;
    arr[5] = tfs.transform.rotation.y;
    arr[6] = tfs.transform.rotation.z;
    arr[7] = tfs.header.stamp.sec;
    arr[8] = tfs.header.stamp.nsec;
    env->ReleaseDoubleArrayElements(dArray, arr, 0);

    return dArray;
  }

JNIEXPORT jboolean JNICALL Java_org_ros_tf2_1ros_Buffer_canTransform__Ljava_lang_String_2Ljava_lang_String_2IILjava_lang_String_2
  (JNIEnv* env, jobject pThis, jstring pTargetFrame, jstring pSourceFrame, jint sec, jint nsec, jstring pErrorMsg){
    // Convert strings
    const char *target_frame = env->GetStringUTFChars(pTargetFrame, 0);
    if(target_frame == NULL){
        return (jboolean)false;
    }
    const char *source_frame = env->GetStringUTFChars(pSourceFrame, 0);
    if(source_frame == NULL){
        return (jboolean)false;
    }

    // Convert time
    ros::Time lookup_time((uint32_t)sec, (uint32_t)nsec);

    std::string error_msg;
    bool out = buffer_core_->canTransform(target_frame, source_frame, lookup_time, &error_msg);

    // Release strings
    env->ReleaseStringUTFChars(pTargetFrame, target_frame);
    env->ReleaseStringUTFChars(pSourceFrame, source_frame);

    // Set error message
    pErrorMsg = env->NewStringUTF(error_msg.c_str());
    return (jboolean)out;
  }

JNIEXPORT jboolean JNICALL Java_org_ros_tf2_1ros_Buffer_canTransform__Ljava_lang_String_2IILjava_lang_String_2IILjava_lang_String_2Ljava_lang_String_2
  (JNIEnv* env, jobject pThis, jstring pTargetFrame, jint pTargetSec, jint pTargetNSec,
   jstring pSourceFrame, jint pSourceSec, jint pSourceNSec, jstring pFixedFrame, jstring pErrorMsg){
    // Convert strings
    const char *target_frame = env->GetStringUTFChars(pTargetFrame, 0);
    if(target_frame == NULL){
        return (jboolean)false;
    }
    const char *source_frame = env->GetStringUTFChars(pSourceFrame, 0);
    if(source_frame == NULL){
        return (jboolean)false;
    }
    const char *fixed_frame = env->GetStringUTFChars(pFixedFrame, 0);
    if(target_frame == NULL){
        return (jboolean)false;
    }

    // Convert times
    ros::Time target_time((uint32_t)pTargetSec, (uint32_t)pTargetNSec);
    ros::Time source_time((uint32_t)pSourceSec, (uint32_t)pSourceNSec);

    std::string error_msg;
    bool out = buffer_core_->canTransform(target_frame, target_time, source_frame, source_time, fixed_frame, &error_msg);

    // Release strings
    env->ReleaseStringUTFChars(pTargetFrame, target_frame);
    env->ReleaseStringUTFChars(pSourceFrame, source_frame);
    env->ReleaseStringUTFChars(pFixedFrame, fixed_frame);

    // Set error message
    pErrorMsg = env->NewStringUTF(error_msg.c_str());
    return (jboolean)out;
  }


JNIEXPORT jstring JNICALL Java_org_ros_tf2_1ros_Buffer_allFramesAsYAML
  (JNIEnv* env, jobject pThis){
    std::string frames = buffer_core_->allFramesAsYAML();
    jstring out;
    out = env->NewStringUTF(frames.c_str());
    return out;
  }

JNIEXPORT jstring JNICALL Java_org_ros_tf2_1ros_Buffer_allFramesAsString
  (JNIEnv* env, jobject pThis){
    std::string frames = buffer_core_->allFramesAsString();
    jstring out;
    out = env->NewStringUTF(frames.c_str());
    return out;
  }

JNIEXPORT jintArray JNICALL Java_org_ros_tf2_1ros_Buffer_getNativeCacheLength
  (JNIEnv* env, jobject pThis){
    ros::Duration d = buffer_core_->getCacheLength();

    // Encode results
    jintArray dArray = env->NewIntArray( 2 );
    jint *arr = env->GetIntArrayElements(dArray, 0);
    
    arr[0] = d.sec;
    arr[1] = d.nsec;
    env->ReleaseIntArrayElements(dArray, arr, 0);

    return dArray;
  }

JNIEXPORT void JNICALL Java_org_ros_tf2_1ros_Buffer_clear
  (JNIEnv* env, jobject pThis){
    buffer_core_->clear();
  }

/*
*
*
* BENCHMARK FUNCTIONS - Move to other file?
*
*
*/


void create_pr2_tree_map(std::map<std::string, std::vector<std::string> >& map){
  std::vector<std::string> vec;
  vec.push_back("base_link");
  map["base_footprint"] = vec; vec.clear();

  std::string a[] = { "bl_caster_rotation_link", "br_caster_rotation_link", "fl_caster_rotation_link", "fr_caster_rotation_link", "torso_lift_link", "torso_lift_motor_screw_link", "base_bellow_link", "base_laser_link" };
  vec.insert(vec.end(), a, a+8);
  map["base_link"] = vec; vec.clear();

  std::string b[] = { "bl_caster_l_wheel_link", "bl_caster_r_wheel_link" };
  vec.insert(vec.end(), b, b+2);
  map["bl_caster_rotation_link"] = vec; vec.clear();

  std::string c[] = { "br_caster_l_wheel_link", "br_caster_r_wheel_link" };
  vec.insert(vec.end(), c, c+2);
  map["br_caster_rotation_link"] = vec; vec.clear();

  std::string d[] = { "fl_caster_l_wheel_link", "fl_caster_r_wheel_link" };
  vec.insert(vec.end(), d, d+2);
  map["fl_caster_rotation_link"] = vec; vec.clear();

  std::string e[] = { "fr_caster_l_wheel_link", "fr_caster_r_wheel_link" };
  vec.insert(vec.end(), e, e+2);
  map["fr_caster_rotation_link"] = vec; vec.clear();

  std::string f[] = { "head_pan_link", "imu_link", "l_shoulder_pan_link", "l_torso_lift_side_plate_link", "laser_tilt_mount_link", "r_shoulder_pan_link", "l_torso_lift_side_plate_link" };
  vec.insert(vec.end(), f, f+7);
  map["torso_lift_link"] = vec; vec.clear();

  std::string g[] = { "head_tilt_link" };
  vec.insert(vec.end(), g, g+1);
  map["head_pan_link"] = vec; vec.clear();

  std::string h[] = { "head_plate_frame" };
  vec.insert(vec.end(), h, h+1);
  map["head_tilt_link"] = vec; vec.clear();

  std::string i[] = { "projector_wg6802418_frame", "sensor_mount_link", "head_mount_link" };
  vec.insert(vec.end(), i, i+3);
  map["head_plate_frame"] = vec; vec.clear();

  std::string j[] = { "projector_wg6802418_child_frame" };
  vec.insert(vec.end(), j, j+1);
  map["projector_wg6802418_frame"] = vec; vec.clear();

  std::string k[] = { "double_stereo_link", "high_def_frame" };
  vec.insert(vec.end(), k, k+2);
  map["sensor_mount_link"] = vec; vec.clear();

  std::string l[] = { "narrow_stereo_link", "wide_stereo_link" };
  vec.insert(vec.end(), l, l+2);
  map["double_stereo_link"] = vec; vec.clear();

  std::string m[] = { "narrow_stereo_optical_frame", "narrow_stereo_l_stereo_camera_frame" };
  vec.insert(vec.end(), m, m+2);
  map["narrow_stereo_link"] = vec; vec.clear();

  std::string n[] = { "narrow_stereo_l_stereo_camera_optical_frame", "narrow_stereo_r_stereo_camera_frame" };
  vec.insert(vec.end(), n, n+2);
  map["narrow_stereo_l_stereo_camera_frame"] = vec; vec.clear();

  std::string o[] = { "narrow_stereo_r_stereo_camera_optical_frame" };
  vec.insert(vec.end(), o, o+1);
  map["narrow_stereo_r_stereo_camera_frame"] = vec; vec.clear();

  std::string p[] = { "wide_stereo_optical_frame", "wide_stereo_l_stereo_camera_frame" };
  vec.insert(vec.end(), p, p+2);
  map["wide_stereo_link"] = vec; vec.clear();

  std::string q[] = { "wide_stereo_l_stereo_camera_optical_frame", "wide_stereo_r_stereo_camera_frame" };
  vec.insert(vec.end(), q, q+2);
  map["wide_stereo_l_stereo_camera_frame"] = vec; vec.clear();

  std::string r[] = { "wide_stereo_r_stereo_camera_optical_frame" };
  vec.insert(vec.end(), r, r+1);
  map["wide_stereo_r_stereo_camera_frame"] = vec; vec.clear();

  std::string s[] = { "wide_stereo_r_stereo_camera_optical_frame" };
  vec.insert(vec.end(), s, s+1);
  map["wide_stereo_r_stereo_camera_frame"] = vec; vec.clear();

  std::string t[] = { "high_def_optical_frame" };
  vec.insert(vec.end(), t, t+1);
  map["high_def_frame"] = vec; vec.clear();

  std::string u[] = { "head_mount_prosilica_link", "head_mount_kinect_ir_link" };
  vec.insert(vec.end(), u, u+2);
  map["head_mount_link"] = vec; vec.clear();

  std::string v[] = { "head_mount_prosilica_optical_frame" };
  vec.insert(vec.end(), v, v+1);
  map["head_mount_prosilica_link"] = vec; vec.clear();

  std::string w[] = { "head_mount_kinect_ir_optical_frame", "head_mount_kinect_rgb_link" };
  vec.insert(vec.end(), w, w+2);
  map["head_mount_kinect_ir_link"] = vec; vec.clear();

  std::string x[] = { "head_mount_kinect_rgb_optical_frame" };
  vec.insert(vec.end(), x, x+1);
  map["head_mount_kinect_rgb_link"] = vec; vec.clear();

  std::string y[] = { "l_shoulder_lift_link" };
  vec.insert(vec.end(), y, y+1);
  map["l_shoulder_pan_link"] = vec; vec.clear();

  std::string z[] = { "l_upper_arm_roll_link" };
  vec.insert(vec.end(), z, z+1);
  map["l_shoulder_lift_link"] = vec; vec.clear();

  std::string aa[] = { "l_upper_arm_link" };
  vec.insert(vec.end(), aa, aa+1);
  map["l_upper_arm_roll_link"] = vec; vec.clear();

  std::string ab[] = { "l_elbow_flex_link" };
  vec.insert(vec.end(), ab, ab+1);
  map["l_upper_arm_link"] = vec; vec.clear();

  std::string ac[] = { "l_forearm_roll_link" };
  vec.insert(vec.end(), ac, ac+1);
  map["l_elbow_flex_link"] = vec; vec.clear();

  std::string ad[] = { "l_forearm_cam_frame", "l_forearm_link" };
  vec.insert(vec.end(), ad, ad+2);
  map["l_forearm_roll_link"] = vec; vec.clear();

  std::string ae[] = { "l_forearm_cam_optical_frame" };
  vec.insert(vec.end(), ae, ae+1);
  map["l_forearm_cam_frame"] = vec; vec.clear();

  std::string af[] = { "l_wrist_flex_link" };
  vec.insert(vec.end(), af, af+1);
  map["l_forearm_link"] = vec; vec.clear();

  std::string ag[] = { "l_wrist_roll_link" };
  vec.insert(vec.end(), ag, ag+1);
  map["l_wrist_flex_link"] = vec; vec.clear();

  std::string ah[] = { "l_gripper_palm_link" };
  vec.insert(vec.end(), ah, ah+1);
  map["l_wrist_roll_link"] = vec; vec.clear();

  std::string ai[] = { "l_gripper_r_finger_link", "l_gripper_tool_frame", "l_gripper_l_finger_link", "l_gripper_led_frame", "l_gripper_motor_accelerometer_link", "l_gripper_motor_slider_link" };
  vec.insert(vec.end(), ai, ai+6);
  map["l_gripper_palm_link"] = vec; vec.clear();

  std::string aj[] = { "l_gripper_r_finger_tip_link" };
  vec.insert(vec.end(), aj, aj+1);
  map["l_gripper_r_finger_link"] = vec; vec.clear();

  std::string ak[] = { "l_gripper_l_finger_tip_frame" };
  vec.insert(vec.end(), ak, ak+1);
  map["l_gripper_r_finger_tip_link"] = vec; vec.clear();

  std::string al[] = { "l_gripper_l_finger_tip_link" };
  vec.insert(vec.end(), al, al+1);
  map["l_gripper_l_finger_link"] = vec; vec.clear();

  std::string am[] = { "l_gripper_motor_screw_link" };
  vec.insert(vec.end(), am, am+1);
  map["l_gripper_motor_slider_link"] = vec; vec.clear();

  std::string an[] = { "laser_tilt_link" };
  vec.insert(vec.end(), an, an+1);
  map["laser_tilt_mount_link"] = vec; vec.clear();

  std::string ao[] = { "r_shoulder_lift_link" };
  vec.insert(vec.end(), ao, ao+1);
  map["r_shoulder_pan_link"] = vec; vec.clear();

  std::string ap[] = { "r_upper_arm_roll_link" };
  vec.insert(vec.end(), ap, ap+1);
  map["r_shoulder_lift_link"] = vec; vec.clear();

  std::string aq[] = { "r_upper_arm_link" };
  vec.insert(vec.end(), aq, aq+1);
  map["r_upper_arm_roll_link"] = vec; vec.clear();

  std::string ar[] = { "r_elbow_flex_link" };
  vec.insert(vec.end(), ar, ar+1);
  map["r_upper_arm_link"] = vec; vec.clear();

  std::string as[] = { "r_forearm_roll_link" };
  vec.insert(vec.end(), as, as+1);
  map["r_elbow_flex_link"] = vec; vec.clear();

  std::string at[] = { "r_forearm_cam_frame", "r_forearm_link" };
  vec.insert(vec.end(), at, at+2);
  map["r_forearm_roll_link"] = vec; vec.clear();

  std::string au[] = { "r_forearm_cam_optical_frame" };
  vec.insert(vec.end(), au, au+1);
  map["r_forearm_cam_frame"] = vec; vec.clear();

  std::string av[] = { "r_wrist_flex_link" };
  vec.insert(vec.end(), av, av+1);
  map["r_forearm_link"] = vec; vec.clear();

  std::string aw[] = { "r_wrist_roll_link" };
  vec.insert(vec.end(), aw, aw+1);
  map["r_wrist_flex_link"] = vec; vec.clear();

  std::string ax[] = { "r_gripper_palm_link" };
  vec.insert(vec.end(), ax, ax+1);
  map["r_wrist_roll_link"] = vec; vec.clear();

  std::string ay[] = { "r_gripper_r_finger_link", "r_gripper_tool_frame", "r_gripper_l_finger_link", "r_gripper_led_frame", "r_gripper_motor_accelerometer_link", "r_gripper_motor_slider_link" };
  vec.insert(vec.end(), ay, ay+6);
  map["r_gripper_palm_link"] = vec; vec.clear();

  std::string az[] = { "r_gripper_r_finger_tip_link" };
  vec.insert(vec.end(), az, az+1);
  map["r_gripper_r_finger_link"] = vec; vec.clear();

  std::string ba[] = { "r_gripper_l_finger_tip_frame" };
  vec.insert(vec.end(), ba, ba+1);
  map["r_gripper_r_finger_tip_link"] = vec; vec.clear();

  std::string bb[] = { "r_gripper_l_finger_tip_link" };
  vec.insert(vec.end(), bb, bb+1);
  map["r_gripper_l_finger_link"] = vec; vec.clear();

  std::string bc[] = { "r_gripper_motor_screw_link" };
  vec.insert(vec.end(), bc, bc+1);
  map["r_gripper_motor_slider_link"] = vec; vec.clear();
}

JNIEXPORT void JNICALL Java_org_ros_tf2_1ros_Buffer_loadPR2Tree
  (JNIEnv* env, jobject pThis){
    bool all_static = false;

    std::map<std::string, std::vector<std::string> > map;
    create_pr2_tree_map(map);

    geometry_msgs::TransformStamped tfs;
    tfs.header.seq = 100;
    tfs.transform.translation.x = 1.0;
    tfs.transform.rotation.w = 1.0;
    std::string authority = "test";

    double dt = 0.01;
    for(double t = 10.0; t < 20.0; t=t+dt){
      ros::Time rt(t);
      tfs.header.stamp = rt;
      // Insert PR2 map at time rt
      for (std::map<std::string, std::vector<std::string> >::iterator it=map.begin(); it!=map.end(); ++it){
        tfs.header.frame_id = it->first;
        for (std::vector<std::string>::iterator vit = it->second.begin() ; vit != it->second.end(); ++vit){
          tfs.child_frame_id = *vit;
          buffer_core_->setTransform(tfs, authority, all_static);
        }
      }
    }

    // Once more to know we have the end time
    ros::Time rt(20.0);
    tfs.header.stamp = rt;
    // Insert PR2 map
    for (std::map<std::string, std::vector<std::string> >::iterator it=map.begin(); it!=map.end(); ++it){
      tfs.header.frame_id = it->first;
      for (std::vector<std::string>::iterator vit = it->second.begin() ; vit != it->second.end(); ++vit){
        tfs.child_frame_id = *vit;
        buffer_core_->setTransform(tfs, authority, all_static);
      }
    }
  }


JNIEXPORT jobjectArray JNICALL Java_org_ros_tf2_1ros_Buffer_getPR2FrameIds
  (JNIEnv* env, jobject pThis){
    std::map<std::string, std::vector<std::string> > map;
    create_pr2_tree_map(map);

    std::set<std::string> out_set;
    for (std::map<std::string, std::vector<std::string> >::iterator it=map.begin(); it!=map.end(); ++it){
      out_set.insert(it->first);
      for (std::vector<std::string>::iterator vit = it->second.begin() ; vit != it->second.end(); ++vit){
        out_set.insert(*vit);
      }
    }

    std::vector<std::string> set_vector(out_set.begin(), out_set.end());  // Want constant block of elements

    // Return to JAVA as a String[]

    jobjectArray ret = (jobjectArray)env->NewObjectArray(set_vector.size(), env->FindClass("java/lang/String"), env->NewStringUTF(""));

    for(size_t i = 0; i<set_vector.size(); i++){
      env->SetObjectArrayElement(ret, i, env->NewStringUTF(set_vector[i].c_str()));
    }

    return ret;
  }