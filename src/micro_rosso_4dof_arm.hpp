#ifndef __4DOF_arm_h
#define __4DOF_arm_h
#include "micro_rosso.h"
#include <Servo.h>

#include <std_srvs/srv/set_bool.h>

/* servo_top
 *    ↓
 *    O----C
 *    |  ↑linkage_top
 *    | ←linkage_bottom
 *   ̲̲_O̲_ ←servo_bottom
 */
class Three_DOF_Arm_with_endeffector {
public:
  Three_DOF_Arm_with_endeffector();
  static bool
  /**
   * @param servo_* Servo objects
   * @param linkage_*_length size of arm rigid components
   * @param servo_*_calibration servo's actual angle range
   *
   */
  setup(Servo *servo_bottom, Servo *servo_top,
    Servo *servo_wrist, Servo *servo_endeffector,
    uint linkage_bottom_length, uint linkage_top_length,
        std::pair<float, float> servo_top_limits = {0, 180},
        std::pair<float, float> servo_bottom_limits = {0, 180},
        std::pair<float, float> servo_wrist_limits = {0, 180},
        std::pair<float, float> servo_endeffector_limits = {0, 180},
        const char *ros_namespace = "/");

  // TODO errors go to ros2 log
  static void move_arm_to(float x, float y);
  static void tilt_endeffector_to(float angle);
  static void move_endeffector(float position);

  static void home_arm();
  static void stash_endeffector(bool stash = true);
  // static void attach_emergency_stop(); //NOTE ineffective unless we can power
  // down the servos somehow

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
  static void retract_endeffector_srv_cb(const void *req, void *res);
};

#endif // __4DOF_arm_h
