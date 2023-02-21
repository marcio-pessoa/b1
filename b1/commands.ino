/* commands.ino, b1 Mark I - Self Balancing Robot, Arduino commands sketch file
 *
 * Copyright 2019-2023 Marcio Pessoa
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

/// @brief Shows help message
/// @param letter
void CommandM100(char letter = 0) {
  if (letter == 'G' || letter == 0) {
    echoln(F("G21\tMove forward"));
    echoln(F("G22\tMove Backward"));
    echoln(F("G41\tTurn Left"));
    echoln(F("G42\tTurn Right"));
  }
  if (letter == 'M' || letter == 0) {
    echoln(F("M00\tCompulsory stop"));
    echoln(F("M92\tSystem information"));
    echoln(F("M93\tBeep"));
    echoln(F("M94\tReturn angle (degrees)"));
    echoln(F("M99\tReset system"));
    echoln(F("M100\tThis help message"));
  }
}

/// @brief Move forward
void CommandG21() { pid_controller.front = 250; }

/// @brief Move Backward
void CommandG22() { pid_controller.back = -250; }

/// @brief Turn Left
void CommandG41() { pid_controller.left = 1; }

/// @brief Turn Right
void CommandG42() { pid_controller.right = 1; }

/// @brief Compulsory stop
void CommandM00() {
  pid_controller.front = 0;
  pid_controller.back = 0;
  pid_controller.left = 0;
  pid_controller.right = 0;
}

/// @brief System information
void CommandM92() {
  echoln(b1.version());
  if (debug || (millis() < 100)) {
    echoln(b1.owner());
    echoln(b1.compiled());
    echoln(b1.license());
    echoln(b1.website());
    echoln(b1.contact());
  }
}

void CommandM93() { buzzer(); }

void CommandM94() { echoln(String(pid_controller.angle)); }

/// @brief Reset
void CommandM99() {
  echoln("Reseting...\n");
  b1.reset();
}

void Command0() { echoln(F("Unknown command")); }
