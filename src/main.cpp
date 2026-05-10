#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <Servo.h>

#include "micro_rosso.h"

#include "ticker.h"
Ticker ticker;

#include "sync_time.h"
SyncTime sync_time;

#include "ros_status.h"
RosStatus ros_status;

const int ledpin = 25;
Adafruit_NeoPixel pixel(1, ledpin, NEO_GRB + NEO_KHZ800);

#include "micro_rosso_4dof_arm.hpp"
Three_DOF_Arm_with_endeffector four_DOF_arm;

Servo servo_top;
Servo servo_bottom;
Servo servo_wrist;
Servo servo_gripper;

void led_callback(int64_t last_call_time) {
  static bool status;
  pixel.setPixelColor(0, pixel.Color(0, 20 * status, 0));
  pixel.show();
  status = !status;
}

void setup() {
  D_println("Booting...");

  // DS3240 limits:  min: 500 , max: 2500
  // Values constrained by mounting position
  servo_bottom.attach(3, 890, 2480);
  constexpr std::pair<float, float> servo_bottom_calibration = {215, 0}; // degrees
  servo_top.attach(4, 650, 2340);
  constexpr std::pair<float, float> servo_top_calibration = {223, 0}; // degrees
  servo_wrist.attach(1, 950, 2500);
  constexpr std::pair<float, float> servo_wrist_calibration = {125, -90}; // degrees
  servo_gripper.attach(2, 500, 2500);
  constexpr std::pair<float, float> servo_gripper_calibration = {180, 0}; // degrees

  pixel.begin();
  pixel.clear();
  D_print("Setting up transport... ");
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  //
  // while (true) {
  //   for (float i = 0; i <= 180; i += 0.1) {
  //     servo_bottom.write(i);
  //     delay(5);
  //     digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  //   }
  //   for (int i = 180; i >= 0; i--) {
  //     servo_bottom.write(i);
  //     delay(50);
  //     digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  //   }
  // }
  //
  if (!micro_rosso::setup("my_node_name"))
    D_println("FAIL micro_rosso.setup()");

  if (!ticker.setup())
    D_println("FAIL ticker.setup()");
  ticker.timer_tick.callbacks.push_back(&led_callback);

  if (!sync_time.setup())
    D_println("FAIL sync_time.setup()");

  if (!ros_status.setup())
    D_println("FAIL ros_status.setup()");

  const uint32_t linkage_bottom_length_mm = 150;
  const uint32_t linkage_top_length_mm = 150;
  if (!four_DOF_arm.setup(&servo_bottom, &servo_top, &servo_wrist, &servo_gripper, linkage_bottom_length_mm, linkage_top_length_mm,
                         servo_top_calibration, servo_bottom_calibration, servo_wrist_calibration, servo_gripper_calibration))
    D_println("FAIL two_DOF_arm.setup()");

  D_println("Boot completed.");
}

void loop() { micro_rosso::loop(); }
