#ifndef BENTO_BOX_ARM_SERVO_LIMITS_HPP
#define BENTO_BOX_ARM_SERVO_LIMITS_HPP
#include <utility>

// Values constrained by hardware
// (this here does anti-noclipping into self)

// PWM signal range for servos:
// DS3240 PWM limits:  min: 500 , max: 2500
std::pair<int, int> servo_bottom_range = {890, 2480};
std::pair<int, int> servo_top_range = {650, 2340};
std::pair<int, int> servo_wrist_range = {950, 2500};
std::pair<int, int> servo_gripper_range = {550, 2350};

// what linkage angles (in degrees) do those limits map to:
std::pair<float, float> servo_bottom_calibration = {215, 0};
std::pair<float, float> servo_top_calibration = {223, 0};
std::pair<float, float> servo_wrist_calibration = {125, -90};
std::pair<float, float> servo_gripper_calibration = {45, 0};

#endif // BENTO_BOX_ARM_SERVO_LIMITS_HPP
