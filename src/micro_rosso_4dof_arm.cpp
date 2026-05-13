#include "micro_rosso_4dof_arm.hpp"
#include "micro_rosso.h"
#include "rclc_checks.hpp"

#include <micro_ros_utilities/string_utilities.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <string>

// arm control
subscriber_descriptor sdescriptor_arm_absolute_control;
subscriber_descriptor sdescriptor_arm_relative_control;
geometry_msgs__msg__Point msg_arm_position;

service_descriptor srvdescriptor_home_srv;
std_srvs__srv__Trigger_Request home_srv_request;
std_srvs__srv__Trigger_Response home_srv_response;

geometry_msgs__msg__Point _goal_pos;
geometry_msgs__msg__Point _actual_pos;

// wrist control
subscriber_descriptor sdescriptor_wrist_relative_control;
subscriber_descriptor sdescriptor_wrist_absolute_control;
std_msgs__msg__Float64 msg_wrist_angle;

std_msgs__msg__Float64 _goal_wrist_angle;
std_msgs__msg__Float64 _actual_wrist_angle;
std_msgs__msg__Float64 _wrist_horizontal_offset_angle;

// endeffector control
service_descriptor srvdescriptor_stash_endeffector_srv;
std_srvs__srv__SetBool_Request stash_endeffector_srv_request;
std_srvs__srv__SetBool_Response stash_endeffector_srv_response;

service_descriptor srvdescriptor_endeffector_hold_angle_srv;
std_srvs__srv__SetBool_Request endeffector_hold_angle_srv_request;
std_srvs__srv__SetBool_Response endeffector_hold_angle_srv_response;

subscriber_descriptor sdescriptor_endeffector_relative_control;
subscriber_descriptor sdescriptor_endeffector_absolute_control;
std_msgs__msg__Float64 msg_endeffector_state;

std_msgs__msg__Float64 _actual_endeffector_state;
bool endeffector_hold_angle;

// joint states
publisher_descriptor pdescriptor_joint_states;
sensor_msgs__msg__JointState msg_joint_states;

// local vars
Servo *_servo[joints];
std::pair<float, float> _servo_limits[joints];
uint _linkage_bottom_length;
uint _linkage_top_length;
double _actual_angles[joints];

Three_DOF_Arm_with_endeffector::Three_DOF_Arm_with_endeffector() {
  // prepare joint states msg
  msg_joint_states.name.size = joints;
  static rosidl_runtime_c__String string_storage[joints]; // allocate storage for String array
  std::string names[joints] = {"arm_base_joint", "arm_elbow_joint", "arm_wrist_joint", "arm_endeffector_joint"};
  msg_joint_states.name.data = string_storage;

  msg_joint_states.position.size = joints;
  msg_joint_states.position.capacity = joints * sizeof(std_msgs__msg__Float64);

  for (int i = 0; i < joints; i++) {
    // populate message strings
    msg_joint_states.name.capacity += names[i].length() * sizeof(char) + sizeof(rosidl_runtime_c__String);
    msg_joint_states.name.data[i] = micro_ros_string_utilities_set(msg_joint_states.name.data[i], names[i].c_str());

    _actual_angles[i] = 0.0; // zero out angles
  }

  // rclc messages are zeroed by default, no need to initialize everything else
}

bool Three_DOF_Arm_with_endeffector::setup(
    Servo *servo_bottom, Servo *servo_top, Servo *servo_wrist, Servo *servo_endeffector,
    const uint linkage_bottom_length, const uint linkage_top_length, const std::pair<float, float> &servo_top_limits,
    const std::pair<float, float> &servo_bottom_limits, const std::pair<float, float> &servo_wrist_limits,
    const std::pair<float, float> &servo_endeffector_limits, const char *ros_namespace) {
  // TODO namespace functionality
  if (ros_namespace != nullptr) {
    sdescriptor_arm_absolute_control.type_support =
        const_cast<rosidl_message_type_support_t *>(ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point));
    sdescriptor_arm_absolute_control.topic_name = "/arm_control_absolute";
    sdescriptor_arm_absolute_control.msg = &msg_arm_position;
    sdescriptor_arm_absolute_control.callback = &absolute_coordinate_control_cb;
    micro_rosso::subscribers.push_back(&sdescriptor_arm_absolute_control);
    // NOTE both share the same message storage, don't know if that's gonna
    // become an issue
    sdescriptor_arm_relative_control.type_support =
        const_cast<rosidl_message_type_support_t *>(ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Point));
    sdescriptor_arm_relative_control.topic_name = "/arm_control_relative";
    sdescriptor_arm_relative_control.msg = &msg_arm_position;
    sdescriptor_arm_relative_control.callback = &relative_coordinate_control_cb;
    micro_rosso::subscribers.push_back(&sdescriptor_arm_relative_control);

    sdescriptor_wrist_relative_control.type_support =
        const_cast<rosidl_message_type_support_t *>(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64));
    sdescriptor_wrist_relative_control.topic_name = "/wrist_control_relative";
    sdescriptor_wrist_relative_control.msg = &msg_wrist_angle;
    sdescriptor_wrist_relative_control.callback = &relative_wrist_angle_control_cb;
    micro_rosso::subscribers.push_back(&sdescriptor_wrist_relative_control);
    sdescriptor_wrist_absolute_control.type_support =
        const_cast<rosidl_message_type_support_t *>(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64));
    sdescriptor_wrist_absolute_control.topic_name = "/wrist_control_absolute";
    sdescriptor_wrist_absolute_control.msg = &msg_wrist_angle;
    sdescriptor_wrist_absolute_control.callback = &absolute_wrist_angle_control_cb;
    micro_rosso::subscribers.push_back(&sdescriptor_wrist_absolute_control);

    sdescriptor_endeffector_relative_control.type_support =
        const_cast<rosidl_message_type_support_t *>(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64));
    sdescriptor_endeffector_relative_control.topic_name = "/endeffector_control_relative";
    sdescriptor_endeffector_relative_control.msg = &msg_endeffector_state;
    sdescriptor_endeffector_relative_control.callback = &relative_endeffector_control_cb;
    micro_rosso::subscribers.push_back(&sdescriptor_endeffector_relative_control);
    sdescriptor_endeffector_absolute_control.type_support =
        const_cast<rosidl_message_type_support_t *>(ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64));
    sdescriptor_endeffector_absolute_control.topic_name = "/endeffector_control_absolute";
    sdescriptor_endeffector_absolute_control.msg = &msg_endeffector_state;
    sdescriptor_endeffector_absolute_control.callback = &absolute_endeffector_control_cb;
    micro_rosso::subscribers.push_back(&sdescriptor_endeffector_absolute_control);

    srvdescriptor_home_srv.qos = QOS_DEFAULT;
    srvdescriptor_home_srv.type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger);
    srvdescriptor_home_srv.service_name = "/home_arm";
    srvdescriptor_home_srv.request = &home_srv_request;
    srvdescriptor_home_srv.response = &home_srv_response;
    srvdescriptor_home_srv.callback = &home_srv_cb;
    micro_rosso::services.push_back(&srvdescriptor_home_srv);

    srvdescriptor_stash_endeffector_srv.qos = QOS_DEFAULT;
    srvdescriptor_stash_endeffector_srv.type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool);
    srvdescriptor_stash_endeffector_srv.service_name = "/retract_endeffector";
    srvdescriptor_stash_endeffector_srv.request = &stash_endeffector_srv_request;
    srvdescriptor_stash_endeffector_srv.response = &stash_endeffector_srv_response;
    srvdescriptor_stash_endeffector_srv.callback = &stash_endeffector_srv_cb;
    micro_rosso::services.push_back(&srvdescriptor_stash_endeffector_srv);

    srvdescriptor_endeffector_hold_angle_srv.qos = QOS_DEFAULT;
    srvdescriptor_endeffector_hold_angle_srv.type_support = ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool);
    srvdescriptor_endeffector_hold_angle_srv.service_name = "/hold_endeffector_angle";
    srvdescriptor_endeffector_hold_angle_srv.request = &endeffector_hold_angle_srv_request;
    srvdescriptor_endeffector_hold_angle_srv.response = &endeffector_hold_angle_srv_response;
    srvdescriptor_endeffector_hold_angle_srv.callback = &endeffector_hold_angle_srv_cb;
    micro_rosso::services.push_back(&srvdescriptor_endeffector_hold_angle_srv);

    pdescriptor_joint_states.qos = QOS_DEFAULT;
    pdescriptor_joint_states.type_support =
        const_cast<rosidl_message_type_support_t *>(ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState));
    pdescriptor_joint_states.topic_name = "/joint_states";
    micro_rosso::publishers.push_back(&pdescriptor_joint_states);

    micro_rosso::timer_control.callbacks.push_back(control_loop_cb);
    micro_rosso::timer_report.callbacks.push_back(report_cb);
  }
  _servo[BOTTOM] = servo_bottom;
  _servo[TOP] = servo_top;
  _servo[WRIST] = servo_wrist;
  _servo[ENDEFFECTOR] = servo_endeffector;

  _linkage_bottom_length = linkage_bottom_length;
  _linkage_top_length = linkage_top_length;

  _servo_limits[BOTTOM] = servo_bottom_limits;
  _servo_limits[TOP] = servo_top_limits;
  _servo_limits[WRIST] = servo_wrist_limits;
  _servo_limits[ENDEFFECTOR] = servo_endeffector_limits;

  home_arm();              // the sudden movement is suboptimal, but servos are
  stash_endeffector(true); // unidirectional communicators so we have no idea where they are.

  D_println("done.");
  return true;
}

// arm state topic callback
void Three_DOF_Arm_with_endeffector::report_cb(int64_t last_call_time) {
  if (pdescriptor_joint_states.topic_name != nullptr) // TODO && last_reading != reading
  {
    msg_joint_states.position.data = _actual_angles;

    micro_rosso::set_timestamp(msg_joint_states.header.stamp);
    RCNOCHECK(rcl_publish(&pdescriptor_joint_states.publisher, &msg_joint_states, nullptr));
  }
}

void Three_DOF_Arm_with_endeffector::absolute_coordinate_control_cb(const void *msgin) {
  if (sdescriptor_arm_absolute_control.topic_name != nullptr) {
    double vector_length_squared = sq(static_cast<const geometry_msgs__msg__Point *>(msgin)->x) +
                                   sq(static_cast<const geometry_msgs__msg__Point *>(msgin)->y);
    if (vector_length_squared > sq(_linkage_bottom_length + _linkage_top_length)) {
      // Out of range! Clamp values to maximum achievable range.
      double clipping_factor = (_linkage_bottom_length + _linkage_top_length) / sqrt(vector_length_squared);
      _goal_pos.y = static_cast<const geometry_msgs__msg__Point *>(msgin)->y * clipping_factor;
      _goal_pos.x = static_cast<const geometry_msgs__msg__Point *>(msgin)->x * clipping_factor;
    } else {
      _goal_pos.y = static_cast<const geometry_msgs__msg__Point *>(msgin)->y;
      _goal_pos.x = static_cast<const geometry_msgs__msg__Point *>(msgin)->x;
    }
  }
}

void Three_DOF_Arm_with_endeffector::relative_coordinate_control_cb(const void *msgin) {
  if (sdescriptor_arm_absolute_control.topic_name != nullptr) {
    double vector_length_squared = abs(sq(static_cast<const geometry_msgs__msg__Point *>(msgin)->x + _goal_pos.x)) +
                                   abs(sq(static_cast<const geometry_msgs__msg__Point *>(msgin)->y + _goal_pos.y));
    if (vector_length_squared > sq(_linkage_bottom_length + _linkage_top_length)) {
      // Out of range!
      // // Clamp values to maximum achievable range. - Behaviour: arm slowly moves
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
      _goal_pos.y += static_cast<const geometry_msgs__msg__Point *>(msgin)->y;
      _goal_pos.x += static_cast<const geometry_msgs__msg__Point *>(msgin)->x;
    }
  }
}

void Three_DOF_Arm_with_endeffector::absolute_wrist_angle_control_cb(const void *msgin) {
  if (sdescriptor_arm_absolute_control.topic_name != nullptr) {
    // Out of range? clamp!
    if (static_cast<const std_msgs__msg__Float64 *>(msgin)->data >
        max(_servo_limits[WRIST].first, _servo_limits[WRIST].second))
      _goal_wrist_angle.data = max(_servo_limits[WRIST].first, _servo_limits[WRIST].second);
    else if (static_cast<const std_msgs__msg__Float64 *>(msgin)->data <
             min(_servo_limits[WRIST].first, _servo_limits[WRIST].second))
      _goal_wrist_angle.data = min(_servo_limits[WRIST].first, _servo_limits[WRIST].second);
    else
      _goal_wrist_angle.data = static_cast<const std_msgs__msg__Float64 *>(msgin)->data;
  }
}

void Three_DOF_Arm_with_endeffector::relative_wrist_angle_control_cb(const void *msgin) {
  if (sdescriptor_arm_absolute_control.topic_name != nullptr) {
    // Out of range? ignore!
    if (static_cast<const std_msgs__msg__Float64 *>(msgin)->data + _goal_wrist_angle.data >
        max(_servo_limits[WRIST].first, _servo_limits[WRIST].second))
      return;
    else if (static_cast<const std_msgs__msg__Float64 *>(msgin)->data + _goal_wrist_angle.data <
             min(_servo_limits[WRIST].first, _servo_limits[WRIST].second))
      return;
    else
      _goal_wrist_angle.data += static_cast<const std_msgs__msg__Float64 *>(msgin)->data;
  }
}

void Three_DOF_Arm_with_endeffector::absolute_endeffector_control_cb(const void *msgin) {
  if (sdescriptor_arm_absolute_control.topic_name != nullptr) {
    // Out of range? clamp!
    if (static_cast<const std_msgs__msg__Float64 *>(msgin)->data >
        max(_servo_limits[ENDEFFECTOR].first, _servo_limits[ENDEFFECTOR].second))
      _actual_endeffector_state.data = max(_servo_limits[ENDEFFECTOR].first, _servo_limits[ENDEFFECTOR].second);
    else if (static_cast<const std_msgs__msg__Float64 *>(msgin)->data <
             min(_servo_limits[ENDEFFECTOR].first, _servo_limits[ENDEFFECTOR].second))
      _actual_endeffector_state.data = min(_servo_limits[ENDEFFECTOR].first, _servo_limits[ENDEFFECTOR].second);
    else
      _actual_endeffector_state.data = static_cast<const std_msgs__msg__Float64 *>(msgin)->data;
  }
}

void Three_DOF_Arm_with_endeffector::relative_endeffector_control_cb(const void *msgin) {
  if (sdescriptor_arm_absolute_control.topic_name != nullptr) {
    // Out of range? ignore!
    if (static_cast<const std_msgs__msg__Float64 *>(msgin)->data + _actual_endeffector_state.data >
        max(_servo_limits[ENDEFFECTOR].first, _servo_limits[ENDEFFECTOR].second))
      return;
    else if (static_cast<const std_msgs__msg__Float64 *>(msgin)->data + _actual_endeffector_state.data <
             min(_servo_limits[ENDEFFECTOR].first, _servo_limits[ENDEFFECTOR].second))
      return;
    else
      _actual_endeffector_state.data += static_cast<const std_msgs__msg__Float64 *>(msgin)->data;
  }
}

void Three_DOF_Arm_with_endeffector::control_loop_cb(int64_t last_call_time) {
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
  move_arm_to(_actual_pos.x, _actual_pos.y);

  if (_goal_wrist_angle.data != _actual_wrist_angle.data) {
    controller(&_actual_wrist_angle.data, _goal_wrist_angle.data);
  }
  if (endeffector_hold_angle)
    angle_wrist_to(_actual_wrist_angle.data + _wrist_horizontal_offset_angle.data);
  else
    angle_wrist_to(_actual_wrist_angle.data);
  move_endeffector(_actual_endeffector_state.data);
}

void Three_DOF_Arm_with_endeffector::home_srv_cb(const void *req, void *res) {
  auto *response = static_cast<std_srvs__srv__Trigger_Response *>(res);
  home_arm();
  response->success = true;
}

void Three_DOF_Arm_with_endeffector::stash_endeffector_srv_cb(const void *req, void *res) {
  stash_endeffector(static_cast<const std_srvs__srv__SetBool_Request *>(req)->data);
  static_cast<std_srvs__srv__SetBool_Response *>(res)->success = true;
}

void Three_DOF_Arm_with_endeffector::endeffector_hold_angle_srv_cb(const void *req, void *res) {
  endeffector_hold_angle = (static_cast<const std_srvs__srv__SetBool_Request *>(req)->data);
  static_cast<std_srvs__srv__SetBool_Response *>(res)->success = true;
}

void Three_DOF_Arm_with_endeffector::move_arm_to(float x, float y) {
  /*  ⭫  servo_top
   *  ┆  ↓   ↓wrist
   *  y  O---O-C ← endeffector
   *  ┆  | ↑linkage_top
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
  _wrist_horizontal_offset_angle.data = (angle_top - angle_bottom) * RAD_TO_DEG;

  float angle_top_deg = 180 - angle_top * RAD_TO_DEG;
  float angle_bottom_deg = 180 - angle_bottom * RAD_TO_DEG;

  float angle_top_deg_servoscaled = map(angle_top_deg, _servo_limits[TOP].first, _servo_limits[TOP].second, 0, 180);
  float angle_bottom_deg_servoscaled =
      map(angle_bottom_deg, _servo_limits[BOTTOM].first, _servo_limits[BOTTOM].second, 0, 180);

  // clip to servo maximums
  if (angle_top_deg_servoscaled > max(_servo_limits[TOP].first, _servo_limits[TOP].second))
    return;
  if (angle_bottom_deg_servoscaled > max(_servo_limits[BOTTOM].first, _servo_limits[BOTTOM].second))
    return;
  if (angle_top_deg_servoscaled < min(_servo_limits[TOP].first, _servo_limits[TOP].second))
    return;
  if (angle_bottom_deg_servoscaled < min(_servo_limits[BOTTOM].first, _servo_limits[BOTTOM].second))
    return;

  // update joint state publisher values
  _actual_angles[BOTTOM] = angle_bottom;
  _actual_angles[TOP] = angle_top;

  // update servos
  _servo[TOP]->write(static_cast<int>(angle_top_deg_servoscaled));
  _servo[BOTTOM]->write(static_cast<int>(angle_bottom_deg_servoscaled));
}

void Three_DOF_Arm_with_endeffector::angle_wrist_to(float angle) {
  _actual_angles[WRIST] = angle * DEG_TO_RAD;
  int angle_wrist_deg_servoscaled = map(angle, _servo_limits[WRIST].first, _servo_limits[WRIST].second, 0, 180);
  _servo[WRIST]->write(angle_wrist_deg_servoscaled);
}

void Three_DOF_Arm_with_endeffector::move_endeffector(float pos) {
  _actual_angles[ENDEFFECTOR] = pos * DEG_TO_RAD;
  int angle_endeffector_servo_deg_servoscaled =
      map(pos, _servo_limits[ENDEFFECTOR].first, _servo_limits[ENDEFFECTOR].second, 0, 180);
  _servo[ENDEFFECTOR]->write(angle_endeffector_servo_deg_servoscaled);
}

void Three_DOF_Arm_with_endeffector::home_arm() {
  // move the arm away from the chassis
  // our arm kinematics mirror along the y axis
  // -> Q1 = Q2, Q3 = Q4
  int travel_delay = 100;
  if (_goal_pos.x < 0 && _goal_pos.y > 0 || // 1st Quartal
      _goal_pos.x > 0 && _goal_pos.y > 0) { // 2nd Quartal
    if (_goal_pos.y > 10)                   // prevent homing jitter
      _goal_pos.y = 10;
    _goal_pos.x = 0.0;
  } else if (_goal_pos.x > 0 && _goal_pos.y < 0 || // 3rd Quartal
             _goal_pos.x < 0 && _goal_pos.y < 0) { // 4th Quartal
    _goal_pos.y = 30;
    _goal_pos.x = 50;
    travel_delay = 400; // extra long travel
  }
  move_arm_to(_goal_pos.x, _goal_pos.y);
  stash_endeffector(true);

  // wait for arm to move, else it could hit the bottom servo/whatever it's mounted to
  delay(travel_delay);
  // hack: the long delay breaks the motion smoothing system, so we override it
  _actual_pos.x = _goal_pos.x;
  _actual_pos.y = _goal_pos.y;

  // final home position
  // y = 1 (because float imprecision will keep the arm homed at an angle since it is "not zero")
  //       (also a value of 1 makes for a better homing movement)
  // x ~ size of linkages (0 when same length)
  _goal_pos.y = 1;
  _goal_pos.x = (_linkage_top_length - _linkage_bottom_length);
  move_arm_to(_goal_pos.x, _goal_pos.y);
}

void Three_DOF_Arm_with_endeffector::stash_endeffector(bool stash) {
  if (stash) {
    // retract as far back as we can go
    _goal_wrist_angle.data = _servo_limits[WRIST].first;
    angle_wrist_to(_goal_wrist_angle.data);
  } else {
    // 0° is flat
    _goal_wrist_angle.data = 0.f;
    angle_wrist_to(_goal_wrist_angle.data);
  }
}
