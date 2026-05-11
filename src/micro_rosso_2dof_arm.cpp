#include "micro_rosso_2dof_arm.hpp"
#include "micro_rosso.h"
#include "rcl/types.h"
#include "rmw/types.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include <micro_ros_utilities/string_utilities.h>

#include <geometry_msgs/msg/point.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_srvs/srv/trigger.h>

static subscriber_descriptor sdescriptor_arm_absolute_control;
static subscriber_descriptor sdescriptor_arm_relative_control;
static geometry_msgs__msg__Point msg_arm_position;

static publisher_descriptor pdescriptor_joint_states;
static sensor_msgs__msg__JointState msg_joint_states;

static service_descriptor srvdescriptor_home_srv;
std_srvs__srv__Trigger_Request home_srv_request;
std_srvs__srv__Trigger_Response home_srv_response;

static Servo *_servo_top;
static Servo *_servo_bottom;

static geometry_msgs__msg__Point _goal_pos;
static geometry_msgs__msg__Point _actual_pos;
static double _actual_angles[2] = {0.0, 0.0};

static uint _linkage_bottom_length;
static uint _linkage_top_length;

static std::pair<float, float> _servo_top_calibration;
static std::pair<float, float> _servo_bottom_calibration;

#define RCCHECK(fn)                                                                                                    \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    if ((temp_rc != RCL_RET_OK)) {                                                                                     \
      return false;                                                                                                    \
    }                                                                                                                  \
  }
#define RCNOCHECK(fn)                                                                                                  \
  {                                                                                                                    \
    rcl_ret_t temp_rc = fn;                                                                                            \
    (void)temp_rc;                                                                                                     \
  }

// arm state topic callback
void Two_DOF_Arm::report_cb(int64_t last_call_time) {
  if (pdescriptor_joint_states.topic_name != 0) // TODO && last_reading != reading
  {
    msg_joint_states.position.data = _actual_angles;

    micro_rosso::set_timestamp(msg_joint_states.header.stamp);
    rcl_publish(&pdescriptor_joint_states.publisher, &msg_joint_states, NULL);
  }
}

void Two_DOF_Arm::absolute_control_cb(const void *msgin) {
  // static float oldx, oldy;
  // if (sdescriptor_arm_absolute_control.topic_name != NULL &&
  // (((geometry_msgs__msg__Point *)msgin)->x != oldx ||
  if (sdescriptor_arm_absolute_control.topic_name != NULL) {
    double vector_length_squared =
        sq(((geometry_msgs__msg__Point *)msgin)->x) + sq(((geometry_msgs__msg__Point *)msgin)->y);
    if (vector_length_squared > sq(_linkage_bottom_length + _linkage_top_length)) {
      // Out of range! Clip values to maximum achievable range.
      float clipping_factor = (_linkage_bottom_length + _linkage_top_length) / sqrt(vector_length_squared);
      _goal_pos.y = ((geometry_msgs__msg__Point *)msgin)->y * clipping_factor;
      _goal_pos.x = ((geometry_msgs__msg__Point *)msgin)->x * clipping_factor;
    } else {
      _goal_pos.y = ((geometry_msgs__msg__Point *)msgin)->y;
      _goal_pos.x = ((geometry_msgs__msg__Point *)msgin)->x;
    }
  }
  // ((geometry_msgs__msg__Point *)msgin)->y != oldy)) {
}

void Two_DOF_Arm::relative_control_cb(const void *msgin) {
  // static float oldx, oldy;
  // if (sdescriptor_arm_absolute_control.topic_name != NULL &&
  // (((geometry_msgs__msg__Point *)msgin)->x != oldx ||
  // ((geometry_msgs__msg__Point *)msgin)->y != oldy)) {
  if (sdescriptor_arm_absolute_control.topic_name != NULL) {
    double vector_length_squared = abs(sq(((geometry_msgs__msg__Point *)msgin)->x + _goal_pos.x)) +
                                   abs(sq(((geometry_msgs__msg__Point *)msgin)->y + _goal_pos.y));
    if (vector_length_squared > sq(_linkage_bottom_length + _linkage_top_length)) {
      // Out of range!
      // // Clip values to maximum achievable range. - Behaviour: arm slowly
      // moves
      // // to most maximum point (→ arm is parallel to an axis)
      // float clipping_factor = (_linkage_bottom_length + _linkage_top_length)
      // /
      //                         sqrt(vector_length_squared);
      // _goal_pos.y += ((geometry_msgs__msg__Point *)msgin)->y;
      // _goal_pos.y *= clipping_factor;
      // _goal_pos.x += ((geometry_msgs__msg__Point *)msgin)->x;
      // _goal_pos.x *= clipping_factor;

      return; // ignore values - Behaviour: arm stops at maximum
    } else {
      _goal_pos.y += ((geometry_msgs__msg__Point *)msgin)->y;
      _goal_pos.x += ((geometry_msgs__msg__Point *)msgin)->x;
    }
  }
}

void Two_DOF_Arm::control_loop_cb(int64_t last_call_time) {
  // TODO PID
  // TODO motion smoothing
  const float kp = 0.25;
  auto controller = [kp](double *actual_pos, double goal_pos) -> void {
    (*actual_pos) += (goal_pos - (*actual_pos)) * kp;
  };

  if (_goal_pos.x != _actual_pos.x) {
    controller(&_actual_pos.x, _goal_pos.x);
  }
  if (_goal_pos.y != _actual_pos.y) {
    controller(&_actual_pos.y, _goal_pos.y);
  }
  // 2dof arm has no z axis

  move_arm_to(_actual_pos.x, _actual_pos.y);
}

void Two_DOF_Arm::home_srv_cb(const void *req, void *res) {
  std_srvs__srv__Trigger_Response *response = (std_srvs__srv__Trigger_Response *)res;
  home_arm();
  response->success = true;
}

// arm setup - initialize data and home arm
Two_DOF_Arm::Two_DOF_Arm() {
  msg_joint_states.name.size = 2;
  msg_joint_states.name.capacity = (15 + 16) * sizeof(char) + 2 * sizeof(rosidl_runtime_c__String);
  msg_joint_states.position.size = 2;
  msg_joint_states.position.capacity = 2 * sizeof(double);

  static rosidl_runtime_c__String string_storage[2]; // allocate storage for String array. silly but works
  msg_joint_states.name.data = (rosidl_runtime_c__String *)string_storage;
  msg_joint_states.name.data[0] = micro_ros_string_utilities_set(msg_joint_states.name.data[0], "arm_base_joint");
  msg_joint_states.name.data[1] = micro_ros_string_utilities_set(msg_joint_states.name.data[1], "arm_elbow_joint");

  // messages come pre-initialized, no need to initialize everything else
};

// set up micro_rosso and analog pin
bool Two_DOF_Arm::setup(Servo *servo_bottom, Servo *servo_top, uint linkage_bottom_length, uint linkage_top_length,
                        std::pair<float, float> servo_top_calibration, std::pair<float, float> servo_bottom_calibration,
                        const char *ros_namespace) {
  // TODO namespace functionality
  if (ros_namespace != NULL) {
    sdescriptor_arm_absolute_control.type_support =
        (rosidl_message_type_support_t *)ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point);
    sdescriptor_arm_absolute_control.topic_name = "/arm_control_absolute";
    sdescriptor_arm_absolute_control.msg = &msg_arm_position;
    sdescriptor_arm_absolute_control.callback = &absolute_control_cb;
    micro_rosso::subscribers.push_back(&sdescriptor_arm_absolute_control);

    // NOTE both share the same message storage, don't know if that's gonna
    // become an issue
    sdescriptor_arm_relative_control.type_support =
        (rosidl_message_type_support_t *)ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point);
    sdescriptor_arm_relative_control.topic_name = "/arm_control_relative";
    sdescriptor_arm_relative_control.msg = &msg_arm_position;
    sdescriptor_arm_relative_control.callback = &relative_control_cb;
    micro_rosso::subscribers.push_back(&sdescriptor_arm_relative_control);

    srvdescriptor_home_srv.qos = QOS_DEFAULT;
    srvdescriptor_home_srv.type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger);
    srvdescriptor_home_srv.service_name = "/home_arm";
    srvdescriptor_home_srv.request = &home_srv_request;
    srvdescriptor_home_srv.response = &home_srv_response;
    srvdescriptor_home_srv.callback = &home_srv_cb;
    micro_rosso::services.push_back(&srvdescriptor_home_srv);

    pdescriptor_joint_states.qos = QOS_DEFAULT;
    pdescriptor_joint_states.type_support =
        (rosidl_message_type_support_t *)ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState);
    pdescriptor_joint_states.topic_name = "/joint_states";
    micro_rosso::publishers.push_back(&pdescriptor_joint_states);

    micro_rosso::timer_control.callbacks.push_back(control_loop_cb);
    micro_rosso::timer_report.callbacks.push_back(report_cb);
  }
  _servo_bottom = servo_bottom;
  _servo_top = servo_top;
  _linkage_bottom_length = linkage_bottom_length;
  _linkage_top_length = linkage_top_length;
  _servo_top_calibration = servo_top_calibration;
  _servo_bottom_calibration = servo_bottom_calibration;

  home_arm(); // the sudden movement is suboptimal, but servos are
              // unidirectional communicators so we have no idea where they are.

  D_println("done.");
  return true;
}

void Two_DOF_Arm::move_arm_to(float x, float y) {
  /*  ⭫  servo_top
   *  ┆  ↓
   *  y  O----C ← wrist
   *  ┆  |  ↑linkage_top
   *  ┆  | ←linkage_bottom
   *  ┆ _O̲_ ←servo_bottom
   *  0┄┄┄┄┄┄┄┄┄┄┄┄x┄┄┄┄┄┄┄⭬
   */

  float length_servobottom_to_wrist = sqrt(sq(x) + sq(y));
  float angle_servobottom_to_wrist = asin(y / length_servobottom_to_wrist);
  float angle_servobottom_to_servotop =
      acos((sq(length_servobottom_to_wrist) + sq(_linkage_bottom_length) - sq(_linkage_top_length)) /
           (2 * _linkage_bottom_length * length_servobottom_to_wrist));
  float angle_bottom = angle_servobottom_to_wrist + angle_servobottom_to_servotop; //- (2 * PI);
  float angle_top = PI - acos((static_cast<float>(sq(_linkage_bottom_length) + sq(_linkage_top_length)) -
                               sq(length_servobottom_to_wrist)) /
                              static_cast<float>(2 * _linkage_bottom_length * _linkage_top_length));

  float angle_top_deg = 180 - angle_top * RAD_TO_DEG;
  float angle_bottom_deg = 180 - angle_bottom * RAD_TO_DEG;

  float angle_top_deg_servoscaled =
      map(angle_top_deg, _servo_top_calibration.first, _servo_top_calibration.second, 0, 180);
  float angle_bottom_deg_servoscaled =
      map(angle_bottom_deg, _servo_bottom_calibration.first, _servo_bottom_calibration.second, 0, 180);

  // clip to servo maximums
  if (angle_top_deg_servoscaled > max(_servo_top_calibration.first, _servo_top_calibration.second))
    return;
  if (angle_bottom_deg_servoscaled > max(_servo_bottom_calibration.first, _servo_bottom_calibration.second))
    return;
  if (angle_top_deg_servoscaled < min(_servo_top_calibration.first, _servo_top_calibration.second))
    return;
  if (angle_bottom_deg_servoscaled < min(_servo_bottom_calibration.first, _servo_bottom_calibration.second))
    return;

  // update joint state publisher values
  _actual_angles[0] = angle_bottom;
  _actual_angles[1] = angle_top;

  // update servos
  _servo_top->write(angle_top_deg_servoscaled);
  _servo_bottom->write(angle_bottom_deg_servoscaled);
}

void Two_DOF_Arm::home_arm() {
  // // joint angles are both 0deg => y=0
  // _goal_pos.y = _goal_pos.x = 50; // move the arm out of the way
  // move_arm_to(_goal_pos.x, _goal_pos.y);

  // make sure the arm homes flat to chassis
  _goal_pos.y = _goal_pos.x = 0.1;
  move_arm_to(_goal_pos.x, _goal_pos.y);

  // wait for arm to move, else it could hit the bottom servo/whatever it's
  // mounted to
  delay(100);

  // x position ≙ size of linkages
  _goal_pos.x = (_linkage_top_length - _linkage_bottom_length);
  move_arm_to(_goal_pos.x, _goal_pos.y);
}
