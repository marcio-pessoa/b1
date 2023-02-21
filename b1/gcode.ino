/* gcode.ino, b1 Mark I - Self Balancing Robot, G-code parser sketch file
 *
 * Copyright 2019-2023 Marcio Pessoa
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

#define BUFFER_SIZE 48

char buffer[BUFFER_SIZE];
int buffer_pointer = 0;

/*
 *
 * Description
 *   .
 *
 *   ()
 *
 * Parameters
 *   none
 *
 * Returns
 *   void
 */
bool echo(String message) { Serial.print(String(message)); }

/*
 *
 * Description
 *   .
 *
 *   ()
 *
 * Parameters
 *   none
 *
 * Returns
 *   void
 */
bool echoln(String message) { echo(message + "\n"); }

/*
 *
 * Description
 *   .
 *
 *   ()
 *
 * Parameters
 *   none
 *
 * Returns
 *   void
 */
void debug(String message) {
  if (debug_mode) {
    echo(message);
  }
}

/*
 *
 * Description
 *   .
 *
 *   ()
 *
 * Parameters
 *   none
 *
 * Returns
 *   void
 */
void debugln(String message) {
  if (debug_mode) {
    echoln(message);
  }
}

/*
 *
 * Description
 *   .
 *
 *   ()
 *
 * Parameters
 *   none
 *
 * Returns
 *   void
 */
void status(bool i) { echoln(i == false ? F("ok") : F("nok")); }

/*
 *
 * Description
 *   .
 *
 *   ()
 *
 * Parameters
 *   none
 *
 * Returns
 *   void
 */
void GcodeReady() { buffer_pointer = 0; }

/*
 *
 * Description
 *   .
 *
 *   ()
 *
 * Parameters
 *   none
 *
 * Returns
 *   void
 */
void GcodeCheck() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (buffer_pointer < BUFFER_SIZE - 1) {
      buffer[buffer_pointer++] = c;
    }
    if (c == '\n') {
      buffer[buffer_pointer] = 0;
      GCodeParse();
      GcodeReady();
    }
  }
}

/*
 *
 * Description
 *   .
 *
 *   ()
 *
 * Parameters
 *   none
 *
 * Returns
 *   void
 */
float GCodeNumber(char code, float val) {
  char *ptr = buffer;
  while (ptr && *ptr && ptr < buffer + buffer_pointer) {
    if (*ptr == code) {
      return atof(ptr + 1);
    }
    ptr = strchr(ptr, ' ') + 1;
  }
  return val;
}

/*
 *
 * Description
 *   .
 *
 *   ()
 *
 * Parameters
 *   none
 *
 * Returns
 *   void
 */
void GCodeParse() {
  bool retval = false;
  bool skip_status = false;
  char letter = buffer[0];
  byte number = GCodeNumber(letter, -1);
  switch (letter) {
    case 'G':
      switch (number) {
        case 21:
          CommandG21();
          break;
        case 22:
          CommandG22();
          break;
        case 41:
          CommandG41();
          break;
        case 42:
          CommandG42();
          break;
        default:
          Command0();
          break;
      }
      break;
    case 'M':
      switch (number) {
        case 0:
          CommandM00();
          break;
        case 92:
          CommandM92();
          break;
        case 93:
          CommandM93();
          break;
        case 94:
          CommandM94();
          break;
        case 99:
          CommandM99();
          break;
        case 100:
          CommandM100();
          break;
        default:
          Command0();
          break;
      }
      break;
    default:
      if (buffer_pointer > 2) {
        Command0();
      }
      break;
  }
  if (buffer_pointer > 2 && skip_status == false) {
    status(retval);
  }
}
