/* b1.ino, b1 Mark I - Self Balancing Robot, Arduino main sketch file
 *
 * Copyright 2019-2020 Marcio Pessoa
 *
 * This sketch was developed and tested on: Arduino Leonardo
 * To work on other Arduino models, some adaptations may be necessary.
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

#include <Arduino.h>       // Main library
#include <Project.h>       // Basic project definitions
//#include <Blinker.h>       // Blink leds nicely
//#include <Timer.h>         // Timer library with nice features
//#include <Alarm.h>         // Manage alarms
//#include <Temperature.h>   // Temperature Sensors
//#include <KalmanFilter.h>  // Temperature Sensors
//#include <MemoryFree.h>   //
#include "./config.h"      // Configuration
#include <MsTimer2.h>      // internal timer 2
#include <PinChangeInt.h>  // Arduino REV4 as external interrupt
#include <MPU6050.h>       // MPU6050 library
#include <Wire.h>          // IIC communication library

// Project definitions
Project b1("b1",  // Platform
           "I",  // Mark
           "Self Balancing Robot",  // Name
           "0.1",  // Version
           "2020-01-16",  // Version date
           "8",  // Serial number
           "Copyright 2019-2020 Marcio Pessoa",  // Owner
           "GPLv2. There is NO WARRANTY.",  // License
           "https://github.com/marcio-pessoa/b1",  // Website
           "Marcio Pessoa <marcio.pessoa@gmail.com>");  // Contact

// Status LED
//Blinker status_led(led_status_pin);

// Teperature sensor
//Temperature lm35;
//Alarm temperature(60,   // Maximum warning
                  //70,   // Maximum critical
                  //10,   // Minimum warning
                   //5);  // Minimum critical

// Check timer
//Timer health_check(health_check_timer * 1000);

// Sensors timer
//Timer sensors_status(sensors_timer * 1000);

// Power save options
//Timer standby((unsigned long)standby_timer * 60 * 1000, COUNTDOWN);
//bool standby_status = false;
//bool standby_done = false;

MPU6050 mpu6050;     // Instantiate an MPU6050 object; name mpu6050
int16_t ax, ay, az, gx, gy, gz;     //Define three-axis acceleration, three-axis gyroscope variables



//////////////////interrupt speed count/////////////////////////////
#define PinA_left 5  //external interrupt
#define PinA_right 4   //external interrupt

void setup() {
  // join I2C bus
  Wire.begin();
  delay(1500);
  // Serial interface
  Serial.begin(serial_speed);
  // Start up message
  Serial.println("Starting...");
  CommandM92();  // System information
  // Temperature
  //lm35.attach(lm35_pin);
  //temperature.nameWrite("Temperature");
  //temperature.unitWrite(" *C");
  // Random number generator seed
  //pinMode(random_Seed_pin, INPUT);
  //randomSeed(analogRead(random_Seed_pin));
  // G-code ready to receive commands
  //GcodeReady();

  // set the motor control pins to OUTPUT
  pinMode(right_R1, OUTPUT);
  pinMode(right_R2, OUTPUT);
  pinMode(left_L1, OUTPUT);
  pinMode(left_L2, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(PWM_L, OUTPUT);

  // assign the initial state value
  digitalWrite(right_R1, 1);
  digitalWrite(right_R2, 0);
  digitalWrite(left_L1, 0);
  digitalWrite(left_L2, 1);
  analogWrite(PWM_R, 0);
  analogWrite(PWM_L, 0);

  pinMode(PinA_left, INPUT);  //speed encoder input
  pinMode(PinA_right, INPUT);

  pinMode(btn, INPUT);
  pinMode(buz, OUTPUT);

  mpu6050.initialize();                       //initialize MPU6050
  delay(2);

  //5ms; use timer2 to set the timer interrupt (note: using timer2 may affects the PWM output of pin3 pin11)
  MsTimer2::set(5, DSzhongduan);    //5ms; execute the function DSzhongduan once
  MsTimer2::start();    //start interrupt
}

void loop() {
  //SensorsHandler();
  //HealthCheckHandler();
  //NotificationHandler();
  //AxesHandler();
  //PowerHandler();
  //GcodeCheck();

  while (i < 1) {
    button = digitalRead(btn);
    if (button == 0) {
      angle0 = -angle;
      //Serial.println(angle0);
      buzzer();
      i++;
    }
  }

  if (Serial.available()) {
    val = Serial.read();      // assign the value read from the serial port to val
    Serial.println(val);
    // switch statement
    switch (val) {
      case 'F': front = 250; break;       // if val equals F，front=250，car will move forward
      case 'B': back = -250; break;       // go back
      case 'L': left = 1; break;    // urn left
      case 'R': right = 1; break;                         //turn right
      case 'S': front = 0, back = 0, left = 0, right = 0; break;    // stop
      case 'D': Serial.print(angle); break;
    }
  }

  //external interrupt; used to calculate the wheel speed
  attachPinChangeInterrupt(PinA_left, Code_left, CHANGE);          // PinA_left Level change triggers the external interrupt; execute the subfunction Code_left
  attachPinChangeInterrupt(PinA_right, Code_right, CHANGE);       // PinA_right Level change triggers the external interrupt; execute the subfunction Code_right
}
