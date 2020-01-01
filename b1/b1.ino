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

#include <Arduino.h>      // Arduino - Main library
#include <Project.h>      // Basic project definitions
#include <Blinker.h>      // Blink leds nicely
#include <Timer.h>        // Timer library with nice features
#include <Alarm.h>        // Manage alarms
#include <Temperature.h>  // Temperature Sensors
#include "./config.h"     // Configuration
#include <MemoryFree.h>   //

// Project definitions
Project b1("b1",  // Platform
           "I",  // Mark
           "Self Balancing Robot",  // Name
           "0.1",  // Version
           "2020-01-16",  // Version date
           "1",  // Serial number
           "Copyright 2019-2020 Marcio Pessoa",  // Owner
           "GPLv2. There is NO WARRANTY.",  // License
           "https://github.com/marcio-pessoa/b1",  // Website
           "Marcio Pessoa <marcio.pessoa@gmail.com>");  // Contact

// Status LED
Blinker status_led(led_ok_pin);

// Teperature sensor
Temperature lm35;
Alarm temperature(60,   // Maximum warning
                  70,   // Maximum critical
                  10,   // Minimum warning
                   5);  // Minimum critical

// Check timer
Timer health_check(health_check_timer * 1000);

// Sensors timer
Timer sensors_status(sensors_timer * 1000);

// Power save options
Timer standby((unsigned long)standby_timer * 60 * 1000, COUNTDOWN);
bool standby_status = false;
bool standby_done = false;

void setup() {
  // Serial interface
  Serial.begin(serial_speed);
  // Start up message
  Serial.println("Starting...");
  CommandM92();  // System information
  // Temperature
  lm35.attach(lm35_pin);
  temperature.nameWrite("Temperature");
  temperature.unitWrite(" *C");
  // Random number generator seed
  pinMode(random_Seed_pin, INPUT);
  randomSeed(analogRead(random_Seed_pin));
  // G-code ready to receive commands
  GcodeReady();
}

void loop() {
  SensorsHandler();
  HealthCheckHandler();
  NotificationHandler();
  AxesHandler();
  PowerHandler();
  GcodeCheck();
}
