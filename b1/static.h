/* static.ino, b1 Mark I - Self Balancing Robot, Arduino static sketch file
 *
 * Copyright 2019-2023 Marcio Pessoa
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

/// @brief first-order filter
/// @param angle_m inclined angle with x-axis
/// @param gyro_m angle speed of Y-axis
/// @param kalman1 tilt calculation
/// @param angleY_one
/// @param samplingTime tilt calculation
/// @return
float Yiorderfilter(float angle_m, float gyro_m,
                    float kalman1, float angleY_one, float samplingTime)
{
  return kalman1 * angle_m +
         (1 - kalman1) * (angleY_one + gyro_m * samplingTime);
}

/// @brief PD angle loop control
/// @param kp angle loop parameter
/// @param angle
/// @param angle0 mechanical balance angle (ideally 0 degrees)
/// @param kd angle loop parameter
/// @param angle_speed
/// @return
int PD(double kp, float angle, float angle0, double kd, float angle_speed)
{
  return kp * (angle + angle0) + kd * angle_speed;
}
