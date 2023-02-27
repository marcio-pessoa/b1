/* handlers.ino, b1 Mark I - Self Balancing Robot, Arduino handlers sketch file
 *
 * Copyright 2019-2023 Marcio Pessoa
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

/// @brief Check for interrupts from motor encoders
void checkISR() {
  attachPinChangeInterrupt(left_encoder_pin, countLeftISR, CHANGE);
  attachPinChangeInterrupt(right_encoder_pin, countRightISR, CHANGE);
}

/// @brief Left speed encoder count ISR (Interrupt Service Routine).
void countLeftISR() { pid_controller.count_left++; }

/// @brief Right speed encoder count ISR (Interrupt Service Routine).
void countRightISR() { pid_controller.count_right++; }

/// @brief Check calibration button
void checkButton() {
  if (!button.check()) {
    angle_default = -pid_controller.angle;
    Serial.println(angle_default);
    buzzer();
  }
}

/// @brief interrupt
void balancing() {
  sei();  // allow overall interrupt

  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  pid_controller.calculateAngle(ax, ay, az, gx, gy, gz);
  pid_controller.calculatePWM(angle_default);
  pid_controller.run();

  moveMotors();
}

/// @brief determine the motor steering and speed by negative and positive of
/// PWM
void moveMotors() {
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
}
