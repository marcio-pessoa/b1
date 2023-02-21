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
           "2020-01-16",                                // Version date
           "8",                                         // Serial number
           "Copyright 2019-2020 Marcio Pessoa",         // Owner
           "GPLv2. There is NO WARRANTY.",              // License
           "https://github.com/marcio-pessoa/b1",       // Website
           "Marcio Pessoa <marcio.pessoa@gmail.com>");  // Contact

// PID Controller
PIDcontroller pid_controller;

// Motor Driver
HBridge motor_left = HBridge(left_pin1, left_pin2, left_pwm);
HBridge motor_right = HBridge(right_pin1, right_pin2, right_pwm);

// Accelerometer
MPU6050 mpu6050;

// Debounce
Debounce button = Debounce(button_pin);

void setup() {
  // Serial interface
  Serial.begin(serial_speed);

  // join I2C bus
  Wire.begin();
  delay(1500);

  // Start up message
  Serial.println("Starting...");
  // CommandM92();  // System information
  // GcodeReady();  // G-code ready to receive commands

  motor_right.backward(0);
  motor_left.forward(0);

  // speed encoder input
  pinMode(left_encoder, INPUT);
  pinMode(right_encoder, INPUT);

  pinMode(buzzer_pin, OUTPUT);

  mpu6050.initialize();
  delay(2);

  // 5ms; use timer2 to set the timer interrupt (note: using timer2 may affects
  // the PWM output of pin3 pin11)
  MsTimer2::set(5, balancing);  // 5ms; execute the function balancing once
  MsTimer2::start();            // start interrupt
}

void loop() {
  // GcodeCheck();

  if (!button.check()) {
    angle_default = -pid_controller.angle;
    Serial.println(angle_default);
    buzzer();
  }

  // if (Serial.available()) {
  //   // assign the value read from the serial port to val
  //   char val = Serial.read();
  //   Serial.println(val);
  //   // switch statement
  //   switch (val) {
  //     case 'F':
  //       pid_controller.front = 250;
  //       break;  // if val equals F，pid_controller.front=250，car will move
  //               // forward
  //     case 'B':
  //       pid_controller.back = -250;
  //       break;  // go back
  //     case 'L':
  //       pid_controller.left = 1;
  //       break;  // urn left
  //     case 'R':
  //       pid_controller.right = 1;
  //       break;  // turn right
  //     case 'S':
  //       pid_controller.front = 0, pid_controller.back = 0,
  //       pid_controller.left = 0, pid_controller.right = 0;
  //       break;  // stop
  //     case 'D':
  //       Serial.print(pid_controller.angle);
  //       break;
  //   }
  // }

  // external interrupt; used to calculate the wheel speed
  // PinA_left Level change triggers the external interrupt
  attachPinChangeInterrupt(left_encoder, countLeftISR, CHANGE);
  // right_encoder Level change triggers the external interrupt
  attachPinChangeInterrupt(right_encoder, countRightISR, CHANGE);
}

/// @brief Left speed encoder count (Interrupt Service Routine).
void countLeftISR() { pid_controller.count_left++; }

/// @brief Right speed encoder count ISR (Interrupt Service Routine).
void countRightISR() { pid_controller.count_right++; }

/// @brief interrupt
void balancing() {
  sei();  // allow overall interrupt

  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // MPU6050 six axis data

  pid_controller.countPulse();
  pid_controller.angleCalculate(ax, ay, az, gx, gy, gz);
  pid_controller.calculatePWM(angle_default);

  setMotors();

  pid_controller.speed();
  pid_controller.guidance();
}

/// @brief determine the motor steering and speed by negative and positive of
/// PWM
void setMotors() {
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
