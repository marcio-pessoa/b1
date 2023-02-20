/* config.h, b1 Mark I - Self Balancing Robot, Arduino project config file
 *
 * Copyright 2019-2023 Marcio Pessoa
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

// LED
// const byte led_status_pin = 13;

// Temperature
// const byte lm35_pin = 0;

// Random number generator Seed pin
// const byte random_Seed_pin = lm35_pin;

// Timers
// const byte health_check_timer = 2;  // seconds
// const byte sensors_timer = 2;  // seconds
// const byte standby_timer = 1;  // minute

// System status (initial state)
// byte general_status = UNKNOWN;

// Debug mode
bool debug_mode = false;

// Serial speed: 4800, 9600, 14400, 19200, 28800, 38400, 57600, 115200
const unsigned long serial_speed = 9600;

// Move done
// bool done = true;

//
// const float FLIMIT = 340282350000000000000000000000000000000;

const int right_R1 = 8;
const int right_R2 = 12;

const int left_L1 = 7;
const int left_L2 = 6;

const int PWM_R = 10;
const int PWM_L = 9;

const int buz = 11;
const int btn = 13;

/////////////////////// angle parameters
float angle0 = 0;  // mechanical balance angle (ideally 0 degrees)
float Gyro_z;      // Angular angular velocity by gyroscope calculation
/////////////////////// angle parameter

/////////////////////// Kalman_Filter
const float Q_angle = 0.001;  // Covariance of gyroscope noise
const float Q_gyro = 0.003;   // Covariance of gyroscope drift noise
const float R_angle = 0.5;    // Covariance of accelerometer
const char C_0 = 1;
const float deltaTime = 0.005;  // The value of dt is the filter sampling time
const float K1 = 0.05;  // a function containing the Kalman gain is used to
                        // calculate the deviation of the optimal estimate

float angle;
float angle_speed;
float angleY_one;
////////////////////// Kalman_Filter

////////////////////// PID parameter
double kp = 34, ki = 0, kd = 0.62;                 // angle loop parameter
double kp_turn = 24, ki_turn = 0, kd_turn = 0.08;  // steering loop parameter
double setp0 = 0;                                  // angle balance point
float pwm1 = 0, pwm2 = 0;

//////////////////////////////// PI variable parameter
double PI_pwm;
int cc;

////////////////////////////// turning PD
int turnmax, turnmin, turnout;
float Turn_pwm = 0;
int turncc = 0;

// Bluetooth
int front = 0;  // forward variable
int back = 0;   // backward
int left = 0;   // turn left
int right = 0;  // turn right
char val;

int i, button;
