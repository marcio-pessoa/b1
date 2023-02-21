/* handlers.ino, b1 Mark I - Self Balancing Robot, Arduino handlers sketch file
 *
 * Copyright 2019-2023 Marcio Pessoa
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

/// @brief
void buzzer() {
  for (int i = 0; i < 50; i++) {
    digitalWrite(buzzer_pin, HIGH);
    delay(1);
    digitalWrite(buzzer_pin, LOW);
    delay(1);
  }
}
