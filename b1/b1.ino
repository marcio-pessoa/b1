/* b1.ino, b1 Mark I - Self Balancing Robot, Arduino main sketch file
 *
 * Copyright 2019-2023 Marcio Pessoa
 *
 * This sketch was developed and tested on: Arduino Leonardo
 * To work on other Arduino models, some adaptations may be necessary.
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

#include <Arduino.h>      // Main library
#include <Wire.h>         // IIC communication library
#include <Project.h>      // Basic project definitions
#include <MsTimer2.h>     // internal timer 2
#include <PinChangeInt.h> // Arduino REV4 as external interrupt
#include <MPU6050.h>      // MPU6050 library
#include "KalmanFilter.h" // Kalman filter
#include "./static.h"     // Static functions
#include "./config.h"     // Configuration
// #include <Blinker.h>      // Blink leds nicely
// #include <Timer.h>        // Timer library with nice features
// #include <Alarm.h>        // Manage alarms
// #include <Temperature.h>  // Temperature Sensors

// Project definitions
Project b1("b1",                                       // Platform
           "I",                                        // Mark
           "Self Balancing Robot",                     // Name
           "0.1",                                      // Version
           "2020-01-16",                               // Version date
           "8",                                        // Serial number
           "Copyright 2019-2020 Marcio Pessoa",        // Owner
           "GPLv2. There is NO WARRANTY.",             // License
           "https://github.com/marcio-pessoa/b1",      // Website
           "Marcio Pessoa <marcio.pessoa@gmail.com>"); // Contact

// Kalman filter
KalmanFilter kalman(dt, Q_angle, Q_gyro, C_0, R_angle);

// Status LED
// Blinker status_led(led_status_pin);

// Teperature sensor
// Temperature lm35;
// Alarm temperature(60,   // Maximum warning
// 70,   // Maximum critical
// 10,   // Minimum warning
// 5);  // Minimum critical

// Check timer
// Timer health_check(health_check_timer * 1000);

// Sensors timer
// Timer sensors_status(sensors_timer * 1000);

// Power save options
// Timer standby((unsigned long)standby_timer * 60 * 1000, COUNTDOWN);
// bool standby_status = false;
// bool standby_done = false;

MPU6050 mpu6050;                // Instantiate an MPU6050 object; name mpu6050
int16_t ax, ay, az, gx, gy, gz; // Define three-axis acceleration, three-axis gyroscope variables

#define PinA_left 5  // external interrupt, interrupt speed count
#define PinA_right 4 // external interrupt, interrupt speed count

void setup()
{
  // Serial interface
  Serial.begin(serial_speed);
  // join I2C bus
  Wire.begin();
  delay(1500);
  // Start up message
  Serial.println("Starting...");
  // CommandM92(); // System information
  // Temperature
  // lm35.attach(lm35_pin);
  // temperature.nameWrite("Temperature");
  // temperature.unitWrite(" *C");
  // Random number generator seed
  // pinMode(random_Seed_pin, INPUT);
  // randomSeed(analogRead(random_Seed_pin));
  // G-code ready to receive commands
  // GcodeReady();

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

  pinMode(PinA_left, INPUT); // speed encoder input
  pinMode(PinA_right, INPUT);

  pinMode(btn, INPUT);
  pinMode(buz, OUTPUT);

  mpu6050.initialize(); // initialize MPU6050
  delay(2);

  // 5ms; use timer2 to set the timer interrupt (note: using timer2 may affects the PWM output of pin3 pin11)
  MsTimer2::set(5, balancing); // 5ms; execute the function DSzhongduan once
  MsTimer2::start();           // start interrupt
}

void loop()
{
  // SensorsHandler();
  // HealthCheckHandler();
  // NotificationHandler();
  // AxesHandler();
  // PowerHandler();
  // GcodeCheck();

  while (i < 1)
  {
    button = digitalRead(btn);
    if (button == 0)
    {
      angle0 = -angle;
      // Serial.println(angle0);
      buzzer();
      i++;
    }
  }

  if (Serial.available())
  {
    val = Serial.read(); // assign the value read from the serial port to val
    Serial.println(val);
    // switch statement
    switch (val)
    {
    case 'F':
      front = 250;
      break; // if val equals F，front=250，car will move forward
    case 'B':
      back = -250;
      break; // go back
    case 'L':
      left = 1;
      break; // urn left
    case 'R':
      right = 1;
      break; // turn right
    case 'S':
      front = 0, back = 0, left = 0, right = 0;
      break; // stop
    case 'D':
      Serial.print(angle);
      break;
    }
  }

  // external interrupt; used to calculate the wheel speed
  // PinA_left Level change triggers the external interrupt
  attachPinChangeInterrupt(PinA_left, countLeftISR, CHANGE);
  // PinA_right Level change triggers the external interrupt
  attachPinChangeInterrupt(PinA_right, countRightISR, CHANGE);
}

/// @brief Left speed encoder count (Interrupt Service Routine).
void countLeftISR()
{
  count_left++;
}

/// @brief Right speed encoder count ISR (Interrupt Service Routine).
void countRightISR()
{
  count_right++;
}

/// @brief pulse count
void countpluse()
{
  lz = count_left; // assign the value counted by encoder to lz
  rz = count_right;

  count_left = 0; // Clear count quantity
  count_right = 0;

  lpluse = lz;
  rpluse = rz;

  if ((pwm1 < 0) && (pwm2 < 0)) // judge the car’s moving direction; if backward (PWM namely motor voltage is negative), pulse is a negative number.
  {
    rpluse = -rpluse;
    lpluse = -lpluse;
  }
  // if backward (PWM namely motor voltage is positive), pulse is a positive number.
  else if ((pwm1 > 0) && (pwm2 > 0))
  {
    rpluse = rpluse;
    lpluse = lpluse;
  }
  // judge the car’s moving direction; if turn left, right pulse is a positive number; left pulse is a negative number.
  else if ((pwm1 < 0) && (pwm2 > 0))
  {
    rpluse = rpluse;
    lpluse = -lpluse;
  }
  // judge the car’s moving direction; if turn right, right pulse is a negative number; left pulse is a positive number.
  else if ((pwm1 > 0) && (pwm2 < 0))
  {
    rpluse = -rpluse;
    lpluse = lpluse;
  }

  // enter interrupt per 5ms，pulse number plus
  pulseright += rpluse;
  pulseleft += lpluse;
}

/// @brief interrupt
void balancing()
{
  sei();        // allow overall interrupt
  countpluse(); // pulse plus subfunction

  mpu6050.getMotion6(&ax, &ay, &az,
                     &gx, &gy, &gz); // IIC to get MPU6050 six-axis data

  Gyro_z = angle_calculate(ax, ay, az,
                           gx, gy, gz,
                           dt,
                           Q_angle, Q_gyro,
                           R_angle, C_0, K1); // get angle and Kalmam filtering

  anglePWM();

  cc++;
  // 5*8=40，enter PI algorithm of speed per 40ms
  if (cc >= 8)
  {
    speedpiout(setp0);
    cc = 0; // Clear
  }

  turncc++;
  // 20ms; enter PD algorithm of steering
  if (turncc > 4)
  {
    turnspin();
    turncc = 0; // Clear
  }
}

/// @brief PWM end value
void anglePWM()
{
  int PD_pwm = PD(kp, angle, angle0, kd, angle_speed); // angle loop PD control

  pwm2 = -PD_pwm - PI_pwm + Turn_pwm; // assign the end value of PWM to motor
  pwm1 = -PD_pwm - PI_pwm - Turn_pwm;

  // limit PWM value not greater than255
  if (pwm1 > 255)
  {
    pwm1 = 255;
  }
  if (pwm1 < -255)
  {
    pwm1 = -255;
  }
  if (pwm2 > 255)
  {
    pwm2 = 255;
  }
  if (pwm2 < -255)
  {
    pwm2 = -255;
  }

  // if tilt angle is greater than 45°，motor will stop
  if (angle > 45 || angle < -45)
  {
    pwm1 = pwm2 = 0;
  }
  // determine the motor steering and speed by negative and positive of PWM
  if (pwm2 >= 0)
  {
    digitalWrite(left_L1, LOW);
    digitalWrite(left_L2, HIGH);
    analogWrite(PWM_L, pwm2);
  }
  else
  {
    digitalWrite(left_L1, HIGH);
    digitalWrite(left_L2, LOW);
    analogWrite(PWM_L, -pwm2);
  }

  if (pwm1 >= 0)
  {
    digitalWrite(right_R1, LOW);
    digitalWrite(right_R2, HIGH);
    analogWrite(PWM_R, pwm1);
  }
  else
  {
    digitalWrite(right_R1, HIGH);
    digitalWrite(right_R2, LOW);
    analogWrite(PWM_R, -pwm1);
  }
}

/// @brief Tilt calculation.
/// @param ax
/// @param ay
/// @param az
/// @param gx
/// @param gy
/// @param gz
/// @param dt The value of dt is the filter sampling time
/// @param Q_angle
/// @param Q_gyro
/// @param R_angle
/// @param C_0
/// @param K1
float angle_calculate(int16_t ax, int16_t ay, int16_t az,
                      int16_t gx, int16_t gy, int16_t gz,
                      float dt,
                      float Q_angle, float Q_gyro,
                      float R_angle, float C_0, float K1)
{
  // Radial rotation angle calculation formula; negative sign is direction processing
  float sensorAngle = -atan2(ay, az) * (180 / PI);

  // The X-axis angular velocity calculated by the gyroscope;  the negative sign is the direction processing
  float Gyro_x = -gx / 131;

  kalman.run(angle, sensorAngle, Gyro_x);
  angle = kalman.angle;
  angle_speed = kalman.angle_speed;

  float angleAx = -atan2(ax, az) * (180 / PI); // calculate the inclined angle with x-axis

  float Gyro_y = -gy / 131.00; // angle speed of Y-axis

  angleY_one = Yiorderfilter(angleAx, Gyro_y, K1, angleY_one, dt); // first-order filtering

  // rotating angle Z-axis parameter
  return -gz / 131; // angle speed of Z-axis
}

/// @brief speed PI
/// @param step0 angle balance point
void speedpiout(double step0)
{
  float speeds = (pulseleft + pulseright) * 1.0; // speed  pulse value

  pulseright = pulseleft = 0; // clear
  speeds_filterold *= 0.7;    // first-order complementary filtering

  float speeds_filter = speeds_filterold + speeds * 0.3;
  speeds_filterold = speeds_filter;

  positions += speeds_filter;
  positions += front; // Forward control fusion

  positions += back;                             // backward control fusion
  positions = constrain(positions, -3550, 3550); // Anti-integral saturation

  PI_pwm = ki_speed * (step0 - positions) +
           kp_speed * (step0 - speeds_filter); // speed loop control PI
}

/// @brief turning
void turnspin()
{
  int flag = 0; //
  float turnspeed = 0;
  float rotationratio = 0;

  if (left == 1 || right == 1)
  {
    if (flag == 0) // judge the speed before rotate, to increase the flexibility
    {
      turnspeed = (pulseright + pulseleft); // current speed ; express in pulse
      flag = 1;
    }
    if (turnspeed < 0) // speed absolute value
    {
      turnspeed = -turnspeed;
    }
    // if press left key or right key
    if (left == 1 || right == 1)
    {
      turnmax = 3;  // max turning value
      turnmin = -3; // min turning value
    }
    rotationratio = 5 / turnspeed; // speed setting value
    if (rotationratio < 0.5)
    {
      rotationratio = 0.5;
    }

    if (rotationratio > 5)
    {
      rotationratio = 5;
    }
  }
  else
  {
    rotationratio = 0.5;
    flag = 0;
    turnspeed = 0;
  }
  // plus according to direction parameter
  if (left == 1)
  {
    turnout += rotationratio;
    // plus according to direction parameter
  }
  else if (right == 1)
  {
    turnout -= rotationratio;
  }
  else
    turnout = 0;
  if (turnout > turnmax)
    turnout = turnmax; // max value of amplitude
  if (turnout < turnmin)
    turnout = turnmin; // min value of amplitude

  Turn_pwm = -turnout * kp_turn - Gyro_z * kd_turn; // turning PD algorithm control
}

/// @brief
void buzzer()
{
  for (int i = 0; i < 50; i++)
  {
    digitalWrite(buz, HIGH);
    delay(1);
    digitalWrite(buz, LOW);
    delay(1);
  }
  delay(50);
  for (int i = 0; i < 50; i++)
  {
    digitalWrite(buz, HIGH);
    delay(1);
    digitalWrite(buz, LOW);
    delay(1);
  }
}
