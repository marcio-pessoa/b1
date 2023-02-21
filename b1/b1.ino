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
#include <MPU6050.h>        // MPU6050 library
#include <MsTimer2.h>       // internal timer 2
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
HBridge motor_left = HBridge(left_L1, left_L2, PWM_L);
HBridge motor_right = HBridge(right_R1, right_R2, PWM_R);

// Instantiate an MPU6050 object; name mpu6050
MPU6050 mpu6050;

// Define three-axis acceleration, three-axis gyroscope variables
int16_t ax, ay, az, gx, gy, gz;

#define PinA_left 5   // external interrupt, interrupt speed count
#define PinA_right 4  // external interrupt, interrupt speed count

void setup() {
  // Serial interface
  Serial.begin(serial_speed);

  // join I2C bus
  Wire.begin();
  delay(1500);

  // Start up message
  Serial.println("Starting...");
  CommandM92();  // System information
  // GcodeReady();  // G-code ready to receive commands

  motor_right.backward(0);
  motor_left.forward(0);

  pinMode(PinA_left, INPUT);  // speed encoder input
  pinMode(PinA_right, INPUT);

  pinMode(button_pin, INPUT);
  pinMode(buzzer_pin, OUTPUT);

  mpu6050.initialize();  // initialize MPU6050
  delay(2);

  // 5ms; use timer2 to set the timer interrupt (note: using timer2 may affects
  // the PWM output of pin3 pin11)
  MsTimer2::set(5, balancing);  // 5ms; execute the function DSzhongduan once
  MsTimer2::start();            // start interrupt
}

void loop() {
  GcodeCheck();

  // while (i < 1) {
  //   button = digitalRead(button_pin);
  //   if (button == 0) {
  //     angle0 = -pid_controller.angle;
  //     // Serial.println(angle0);
  //     buzzer();
  //     i++;
  //   }
  // }

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
  attachPinChangeInterrupt(PinA_left, countLeftISR, CHANGE);
  // PinA_right Level change triggers the external interrupt
  attachPinChangeInterrupt(PinA_right, countRightISR, CHANGE);
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

  anglePWM();

  speed_counter++;
  // 5*8=40，enter PI algorithm of speed per 40ms
  if (speed_counter >= speed_counter_limit) {
    pid_controller.speedPIout();
    speed_counter = 0;  // Clear
  }

  guidance_counter++;
  // 20ms; enter PD algorithm of steering
  if (guidance_counter > guidance_counter_limit) {
    pid_controller.turnspin();
    guidance_counter = 0;  // Clear
  }
}

/// @brief PWM end value
void anglePWM() {
  // angle loop PD control
  int PD_pwm = pid_controller.kp * (pid_controller.angle + angle0) +
               pid_controller.kd * pid_controller.angle_speed;

  // assign the end value of PWM to motor
  pid_controller.pwm2 =
      -PD_pwm - pid_controller.PI_pwm + pid_controller.Turn_pwm;
  pid_controller.pwm1 =
      -PD_pwm - pid_controller.PI_pwm - pid_controller.Turn_pwm;

  // limit PWM value not greater than255
  if (pid_controller.pwm1 > 255) {
    pid_controller.pwm1 = 255;
  }
  if (pid_controller.pwm1 < -255) {
    pid_controller.pwm1 = -255;
  }
  if (pid_controller.pwm2 > 255) {
    pid_controller.pwm2 = 255;
  }
  if (pid_controller.pwm2 < -255) {
    pid_controller.pwm2 = -255;
  }

  // if tilt angle is greater than 45°，motor will stop
  if (pid_controller.angle > 45 || pid_controller.angle < -45) {
    pid_controller.pwm1 = pid_controller.pwm2 = 0;
  }
  // determine the motor steering and speed by negative and positive of PWM
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

/// @brief
void buzzer() {
  for (int i = 0; i < 50; i++) {
    digitalWrite(buzzer_pin, HIGH);
    delay(1);
    digitalWrite(buzzer_pin, LOW);
    delay(1);
  }
  delay(50);
  for (int i = 0; i < 50; i++) {
    digitalWrite(buzzer_pin, HIGH);
    delay(1);
    digitalWrite(buzzer_pin, LOW);
    delay(1);
  }
}
