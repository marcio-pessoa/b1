/* static.ino, b1 Mark I - Self Balancing Robot, Arduino statics sketch file
 *
 * Copyright 2019-2023 Marcio Pessoa
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

/// @brief first-order filter
/// @param angle_m
/// @param gyro_m
/// @param K1
/// @param angleY_one
/// @param dt
/// @return
float Yiorderfilter(float angle_m, float gyro_m,
                    float K1, float angleY_one, float dt)
{
  return K1 * angle_m + (1 - K1) * (angleY_one + gyro_m * dt);
}
