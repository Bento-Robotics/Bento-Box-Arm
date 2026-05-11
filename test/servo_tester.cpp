#include <Arduino.h>
#include <Servo.h>

#include "servo_limits.hpp"

Servo testing_servo;

int servo_pin = 2;
int servo_min = servo_gripper_range.first;
int servo_max = servo_gripper_range.second;

void setup() {
  // DS3240 limits:  min: 500 , max: 2500
  testing_servo.attach(servo_pin, servo_min, servo_max);
}

void loop() {
  // count upwards
  for (float i = servo_min; i <= servo_max; i++) {
    testing_servo.write(i);
    delay(2);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  delay(10);
  // count downwards
  for (int i = servo_max; i >= servo_min; i--) {
    testing_servo.write(i);
    delay(2);
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  delay(10);
}