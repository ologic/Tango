#include <stdarg.h>
#include <stdio.h>
#include <sstream>
#include <map>
#include <string.h>
#include <errno.h>
#include <vector>
#include <set>
#include <fstream>
#include <android/log.h>

#include "geometry_msgs/TransformStamped.h"
#include "tf2/buffer_core.h"
#include "tf2/exceptions.h"

#include <android_native_app_glue.h>

void log(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    __android_log_vprint(ANDROID_LOG_INFO, "TF2_NDK_EXAMPLE", msg, args);
    va_end(args);
}


// from android samples
/* return current time in seconds */
static double now(void) {

  struct timespec res;
  clock_gettime(CLOCK_REALTIME, &res);
  return res.tv_sec + (double) res.tv_nsec / 1e9;

}


#define LASTERR strerror(errno)

size_t true_random_index(const size_t n){
  const size_t divisor = RAND_MAX/(n);

  size_t k;
  do { k = std::rand() / divisor; } while (k >= n);
  return k;
}



double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

std::set<std::string> unique_frames_from_map(std::map<std::string, std::vector<std::string> >& map){
  std::set<std::string> out_set;
  for (std::map<std::string, std::vector<std::string> >::iterator it=map.begin(); it!=map.end(); ++it){
    out_set.insert(it->first);
    for (std::vector<std::string>::iterator vit = it->second.begin() ; vit != it->second.end(); ++vit){
      out_set.insert(*vit);
    }
  }
  return out_set;
}

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

void insert_pr2_tree(tf2::BufferCore& buffer, std::map<std::string, std::vector<std::string> >& map, ros::Time& t, bool all_static){
  geometry_msgs::TransformStamped tfs;
  tfs.header.stamp = t;
  tfs.header.seq = 100;
  tfs.transform.translation.x = 1.0;
  tfs.transform.rotation.w = 1.0;
  std::string authority = "test";



  // Insert PR2 map
  for (std::map<std::string, std::vector<std::string> >::iterator it=map.begin(); it!=map.end(); ++it){
    tfs.header.frame_id = it->first;
    for (std::vector<std::string>::iterator vit = it->second.begin() ; vit != it->second.end(); ++vit){
      tfs.child_frame_id = *vit;
      buffer.setTransform(tfs, authority, all_static);
    }
  }
}

void lookup_transforms(tf2::BufferCore& buffer, std::set<std::string> frame_ids){
  std::string frame1 = "child";
  std::string frame2 = "parent";
  geometry_msgs::TransformStamped tfs;

  double num_lookups = 1e5;
  double tree_depth = 0.0;
  std::vector<std::string> set_vector(frame_ids.begin(), frame_ids.end());  // Want constant time element indexing
  size_t n_frames = set_vector.size();
  std::stringstream rs;
  rs << num_lookups << " random lookups - on " << n_frames << " total frames";
  log(rs.str().c_str());
  double start = now();
  for(size_t i = 0; i < num_lookups; i++){
    frame1 = set_vector[true_random_index(n_frames)];
    frame2 = set_vector[true_random_index(n_frames)];
    ros::Time lookup_time(fRand(10.0, 20.0));
    try{
       tfs = buffer.lookupTransform(frame1, frame2, lookup_time);
       tree_depth += abs(tfs.transform.translation.x);
    } catch(const tf2::ConnectivityException& e){
      log("CONNECTIVITY EXCEPTION");
    } catch(const tf2::ExtrapolationException& e){
      log("EXTRAPOLATION EXCEPTION");
    } catch(const tf2::LookupException& e){
      log("LOOKUP EXCEPTION");
    } catch (...) {
      log("SUPER BAD EXCEPTION");
    }
  } 

  double end = now();
  double avg_depth = tree_depth / num_lookups;

  std::stringstream st;
  st << "Start time: " << start << "\nEnd time: " << end
  << "\n Diff: " << end - start << "\n Avg: " << (end-start)/num_lookups
  << "\n Avg tf tree depth:" << avg_depth;
  log(st.str().c_str());
}


void ev_loop(android_app *papp) {
  int32_t lr;
    int32_t le;

    int step = 1;
    android_poll_source *ps;

    app_dummy();

    log("starting event loop");

    log("creating buffer_core");

    tf2::BufferCore b_core;

    log("generating pr2 tree map");
    std::map<std::string, std::vector<std::string> > map;
    create_pr2_tree_map(map);

    while (true) {
        lr = ALooper_pollAll(-1, NULL, &le, (void **) &ps);
        if (lr < 0) {
            break;
        }
        if (ps) {
            log("event received");
            if (step == 1) {   
              log("inserting transforms");

              log("inserting pr2 maps into buffer");
              double dt = 0.01;
              for(double t = 10.0; t < 20.0; t=t+dt){
                ros::Time rt(t);
                insert_pr2_tree(b_core, map, rt, false);
              }
              // Once more to know we have the end time
              ros::Time rt(20.0);
              insert_pr2_tree(b_core, map, rt, false);

              std::string all_frames = b_core.allFramesAsString();
              log(all_frames.c_str());
                
              step++;
            } else if(step == 2){
              log("looking up transform");
              std::set<std::string> frame_ids = unique_frames_from_map(map);
              log("set contains:");
              for (std::set<std::string>::iterator it=frame_ids.begin(); it!=frame_ids.end(); ++it){
                log(it->c_str());
              }
              lookup_transforms(b_core, frame_ids);
              step++;
            } else if (step == 3) {
              log("Demo done.");
              step++;
            }
            ps->process(papp, ps);
        }
        if (papp->destroyRequested) {
            log("quitting event loop");
            return;
        }
    }
}

void android_main(android_app *papp) {
    ev_loop(papp);
}
