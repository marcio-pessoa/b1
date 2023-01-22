/* auxiliar.ino, b1 Mark I - Self Balancing Robot, Arduino auxiliar sketch file
 *
 * Copyright 2019-2023 Marcio Pessoa
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

/* isAllDone
 *
 * Description
 *   Checks if all axes is done (not moving).
 *
 *   isAllDone()
 *
 * Parameters
 *   none
 *
 * Returns
 *   bool: 0 - No axis moving.
 *         1 - There are on or more axes moving.
 */
// bool isAllDone() {
// if (x_axis.isDone() and
// y_axis.isDone()) {
// return true;
//}
// else {
// return false;
//}
//}

/* spinCounter
 *
 * Description
 *   Fan spin counter.
 *
 *   spinCounter()
 *
 * Parameters
 *   none
 *
 * Returns
 *   void
 */
// void spinCounter() {
// fan_control.counter();
//}

/////////////////////Hall count/////////////////////////
// left speed encoder count
void Code_left()
{
  count_left++;
}

// right speed encoder count
void Code_right()
{
  count_right++;
}

////////////////////pulse count///////////////////////
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

/////////////////////////////////interrupt ////////////////////////////
void DSzhongduan()
{
  sei();                                                                          // allow overall interrupt
  countpluse();                                                                   // pulse plus subfunction
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);                               // IIC to get MPU6050 six-axis data  ax ay az gx gy gz
  angle_calculate(ax, ay, az, gx, gy, gz, dt, Q_angle, Q_gyro, R_angle, C_0, K1); // get angle and Kalmam filtering
  PD();                                                                           // angle loop PD control
  anglePWM();

  cc++;
  // 5*8=40，enter PI algorithm of speed per 40ms
  if (cc >= 8)
  {
    speedpiout();
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
///////////////////////////////////////////////////////////

/////////////////////////////tilt calculation///////////////////////
void angle_calculate(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz, float dt, float Q_angle, float Q_gyro, float R_angle, float C_0, float K1)
{
  float Angle = -atan2(ay, az) * (180 / PI); // Radial rotation angle calculation formula ; negative sign is direction processing
  Gyro_x = -gx / 131;                        // The X-axis angular velocity calculated by the gyroscope;  the negative sign is the direction processing
  Kalman_Filter(Angle, Gyro_x);              // Kalman Filter
  // rotating angle Z-axis parameter
  Gyro_z = -gz / 131; // angle speed of Z-axis
  // accelz = az / 1604;

  float angleAx = -atan2(ax, az) * (180 / PI); // calculate the inclined angle with x-axis
  Gyro_y = -gy / 131.00;                       // angle speed of Y-axis
  Yiorderfilter(angleAx, Gyro_y);              // first-order filtering
}
////////////////////////////////////////////////////////////////

/////////////////////////////// Kalman Filter
void Kalman_Filter(double angle_m, double gyro_m)
{
  angle += (gyro_m - q_bias) * dt; // prior estimate
  float angle_err = angle_m - angle;

  Pdot[0] = Q_angle - P[0][1] - P[1][0]; // The differential of the covariance of the prior estimate error
  Pdot[1] = -P[1][1];
  Pdot[2] = -P[1][1];
  Pdot[3] = Q_gyro;

  P[0][0] += Pdot[0] * dt; // The integral of the covariance differential of the prior estimate error
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;

  // Intermediate variables in matrix multiplication
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  // denominator
  E = R_angle + C_0 * PCt_0;
  // gain value
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;

  t_0 = PCt_0; // Intermediate variables in matrix multiplication
  t_1 = C_0 * P[0][1];

  P[0][0] -= K_0 * t_0; // Posterior estimation error covariance
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;

  q_bias += K_1 * angle_err;     // Posterior estimate
  angle_speed = gyro_m - q_bias; // The differential of the output value gives the optimal angular velocity
  angle += K_0 * angle_err;      ////Posterior estimation; get the optimal angle
}

/////////////////////first-order filter/////////////////
void Yiorderfilter(float angle_m, float gyro_m)
{
  angleY_one = K1 * angle_m + (1 - K1) * (angleY_one + gyro_m * dt);
}

//////////////////angle PD////////////////////
void PD()
{
  PD_pwm = kp * (angle + angle0) + kd * angle_speed; // PD angle loop control
}

//////////////////speed PI////////////////////
void speedpiout()
{
  float speeds = (pulseleft + pulseright) * 1.0; // speed  pulse value
  pulseright = pulseleft = 0;                    // clear
  speeds_filterold *= 0.7;                       // first-order complementary filtering
  speeds_filter = speeds_filterold + speeds * 0.3;
  speeds_filterold = speeds_filter;
  positions += speeds_filter;
  positions += front;                                                           // Forward control fusion
  positions += back;                                                            // backward control fusion
  positions = constrain(positions, -3550, 3550);                                // Anti-integral saturation
  PI_pwm = ki_speed * (setp0 - positions) + kp_speed * (setp0 - speeds_filter); // speed loop control PI
}
//////////////////speed PI////////////////////

///////////////////////////turning/////////////////////////////////
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
///////////////////////////turning/////////////////////////////////

////////////////////////////PWM end value/////////////////////////////
void anglePWM()
{
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
