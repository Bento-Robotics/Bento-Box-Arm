#ifndef __4DOF_arm_h
#define __4DOF_arm_h
#include "micro_rosso.h"
#include <Servo.h>

#include <geometry_msgs/msg/point.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/float64.h>
#include <std_srvs/srv/set_bool.h>
#include <std_srvs/srv/trigger.h>

#define joints 4

// util for more legible indexing
enum servos { BOTTOM = 0, TOP = 1, WRIST = 2, ENDEFFECTOR = 3 };

/* servo_top
 *    ↓
 *    O----C
 *    |  ↑linkage_top
 *    | ←linkage_bottom
 *   ̲̲_O̲_ ←servo_bottom
 */
class Three_DOF_Arm_with_endeffector {
public:
  // initialize topic data
  Three_DOF_Arm_with_endeffector();

  static bool
  /**
   * @param servo_* Servo objects
   * @param linkage_*_length size of arm rigid components
   * @param servo_*_calibration servo's actual angle range
   * Initialize arm hardware and home arm
   */
  setup(Servo *servo_bottom, Servo *servo_top,
    Servo *servo_wrist, Servo *servo_endeffector,
    uint linkage_bottom_length, uint linkage_top_length,
        const std::pair<float, float> &servo_top_limits = {0, 180},
        const std::pair<float, float> &servo_bottom_limits = {0, 180},
        const std::pair<float, float> &servo_wrist_limits = {0, 180},
        const std::pair<float, float> &servo_endeffector_limits = {0, 180},
        const char *ros_namespace = "/");

  // TODO errors go to ros2 log
  static void move_arm_to(float x, float y);
  static void angle_wrist_to(float angle);
  static void move_endeffector(float position);

  static void home_arm();
  static void stash_endeffector(bool stash = true);
  // static void attach_emergency_stop(); //NOTE ineffective unless we can power down the servos somehow

private:
  static void report_cb(int64_t last_call_time);
  static void absolute_coordinate_control_cb(const void *msgin);
  static void relative_coordinate_control_cb(const void *msgin);
  static void absolute_wrist_angle_control_cb(const void *msgin);
  static void relative_wrist_angle_control_cb(const void *msgin);
  static void absolute_endeffector_control_cb(const void *msgin);
  static void relative_endeffector_control_cb(const void *msgin);
  static void control_loop_cb(int64_t last_call_time);
  static void home_srv_cb(const void *req, void *res);
  static void stash_endeffector_srv_cb(const void *req, void *res);
  static void endeffector_keep_angle_srv_cb(const void *req, void *res);

  typedef void (*cbt)(const void *);
};

#endif // __4DOF_arm_h
