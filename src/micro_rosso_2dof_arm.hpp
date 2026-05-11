#ifndef __2DOF_arm_h
#define __2DOF_arm_h
#include "micro_rosso.h"
#include <Servo.h>

/* servo_top
 *    ↓
 *    O----C
 *    |  ↑linkage_top
 *    | ←linkage_bottom
 *   ̲̲_O̲_ ←servo_bottom
 */
class Two_DOF_Arm {
public:
  Two_DOF_Arm();
  static bool
  /**
   * @param servo_bottom/top Servo objects
   * @param linkage_*_length size of arm rigid components
   * @param servo_*_calibration servo's actual angle range
   *
   */
  setup(Servo *servo_bottom, Servo *servo_top,
    uint linkage_bottom_length, uint linkage_top_length,
        std::pair<float, float> servo_top_calibration = {0, 180},
        std::pair<float, float> servo_bottom_calibration = {0, 180},
        const char *ros_namespace = "/");

  // TODO errors go to ros2 log
  static void move_arm_to(float x, float y);

  static void home_arm();
  // static void attach_emergency_stop(); //NOTE ineffective unless we can power
  // down the servos somehow

private:
  static void report_cb(int64_t last_call_time);
  static void absolute_control_cb(const void *msgin);
  static void relative_control_cb(const void *msgin);
  static void control_loop_cb(int64_t last_call_time);
  static void home_srv_cb(const void *req, void *res);
};

#endif // __2DOF_arm_h
