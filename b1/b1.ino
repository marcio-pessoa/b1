/* b1.ino, b1 Mark I - Self Balancing Robot, Arduino main sketch file
 *
 * Copyright 2019-2023 Marcio Pessoa
 *
 * This sketch was developed and tested on: Arduino Uno
 * To work on other Arduino models, some adaptations may be necessary.
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

#include <Arduino.h>        // Main library
#include <Debounce.h>       // Avoid Bounce Effect Library
#include <MPU6050.h>        // Accelerometer
#include <MsTimer2.h>       // Internal timer 2
#include <PinChangeInt.h>   // Arduino REV4 as external interrupt
#include <Project.h>        // Basic project definitions
#include <Wire.h>           // IIC communication library
#include "./config.h"       // Configuration
#include "./static.h"       // Static functions
#include "HBridge.h"        // Motor Driver library
#include "PIDcontroller.h"  // PID Controller

// Project definitions
Project b1("b1",                                        // Platform
           "I",                                         // Mark
           "Self Balancing Robot",                      // Name
           "0.1",                                       // Version
           "2023-01-16",                                // Version date
           "8",                                         // Serial number
           "Copyright 2019-2023 Marcio Pessoa",         // Owner
           "GPLv2. There is NO WARRANTY.",              // License
           "https://github.com/marcio-pessoa/b1",       // Website
           "Marcio Pessoa <marcio.pessoa@gmail.com>");  // Contact

// PID Controller
PIDcontroller pid_controller;

// Motor Driver
HBridge motor_left = HBridge(left_pin1, left_pin2, left_pwm_pin);
HBridge motor_right = HBridge(right_pin1, right_pin2, right_pwm_pin);

// Accelerometer
MPU6050 mpu6050;
int16_t ax, ay, az, gx, gy, gz;

// Calibration button (debounced)
Debounce button = Debounce(button_pin);
float angle_default = 0;  // mechanical balance angle (ideally 0 degrees)

void setup() {
  // Serial interface
  Serial.begin(serial_speed);

  // join I2C bus
  Wire.begin();
  delay(1500);

  // Start up message
  Serial.println("Starting...");
  CommandM92();  // System information
  GcodeReady();  // G-code ready to receive commands

  motor_right.backward(0);
  motor_left.forward(0);

  // Motor speed encoder input
  pinMode(left_encoder_pin, INPUT);
  pinMode(right_encoder_pin, INPUT);

  pinMode(buzzer_pin, OUTPUT);

  mpu6050.initialize();
  delay(2);

  // Set the timer interrupt
  // note: using timer2 may affects the PWM output of pin3 pin11
  MsTimer2::set(5, balancing);  // 5ms; execute the function balancing once
  MsTimer2::start();            // start interrupt
}

void loop() {
  checkGcode();

  if (!button.check()) {
    angle_default = -pid_controller.angle;
    Serial.println(angle_default);
    buzzer();
  }

  checkISR();
}

/// @brief Check for interrupts from motor encoders
void checkISR() {
  attachPinChangeInterrupt(left_encoder_pin, countLeftISR, CHANGE);
  attachPinChangeInterrupt(right_encoder_pin, countRightISR, CHANGE);
}

/// @brief Left speed encoder count ISR (Interrupt Service Routine).
void countLeftISR() { pid_controller.count_left++; }

/// @brief Right speed encoder count ISR (Interrupt Service Routine).
void countRightISR() { pid_controller.count_right++; }

/// @brief interrupt
void balancing() {
  sei();  // allow overall interrupt

  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  pid_controller.calculateAngle(ax, ay, az, gx, gy, gz);
  pid_controller.calculatePWM(angle_default);
  pid_controller.run();

  moveMotors();
}

/// @brief determine the motor steering and speed by negative and positive of
/// PWM
void moveMotors() {
  if (pid_controller.pwm2 >= 0) {
    motor_left.forward(pid_controller.pwm2);
  } else {
    motor_left.backward(pid_controller.pwm2);
  }

  if (pid_controller.pwm1 >= 0) {
    motor_right.forward(pid_controller.pwm1);
  } else {
    motor_right.backward(pid_controller.pwm1);
  }
}
