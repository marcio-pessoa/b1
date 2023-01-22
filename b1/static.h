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
/// @param K1 tilt calculation
/// @param angleY_one
/// @param dt tilt calculation
/// @return
float Yiorderfilter(float angle_m, float gyro_m,
                    float K1, float angleY_one, float dt)
{
  return K1 * angle_m + (1 - K1) * (angleY_one + gyro_m * dt);
}

/// @brief angle PD
/// @param kp angle loop parameter
/// @param angle
/// @param angle0 mechanical balance angle (ideally 0 degrees)
/// @param kd angle loop parameter
/// @param angle_speed
/// @return
int PD(double kp, float angle, float angle0, double kd, float angle_speed)
{
  return kp * (angle + angle0) + kd * angle_speed; // PD angle loop control
}