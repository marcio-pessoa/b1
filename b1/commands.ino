/* commands.ino, b1 Mark I - Self Balancing Robot, Arduino commands sketch file
 *
 * Copyright 2019-2023 Marcio Pessoa
 *
 * Author: Márcio Pessoa <marcio.pessoa@gmail.com>
 * Contributors: none
 */

/* CommandM100
 *
 * Description
 *   Shows help messages.
 *
 *   CommandM100()
 *
 * Parameters
 *   letter: The command initial letter. It's used to display a specific help
 *           about a letter.
 *           If letter equals to zero, all help messages will be shown.
 *
 * Returns
 *   void
 */
void CommandM100(char letter = 0) {
  // if (letter == 'G' || letter == 0) {
  // echoln(F("G00\tRapid positioning"));
  // echoln(F("G01\tLinear interpolation"));
  //   echoln(F("G02\tCircular interpolation, clockwise"));
  //   echoln(F("G03\tCircular interpolation, counterclockwise"));
  // echoln(F("G06\tDemonstration mode"));
  // echoln(F("G28\tHome axes"));
  // echoln(F("G90\tAbsolute programming"));
  // echoln(F("G91\tIncremental programming"));
  // echoln(F("G132\tCalibrate axes"));
  //}
  if (letter == 'M' || letter == 0) {
    // echoln(F("M0\tCompulsory stop"));
    // echoln(F("M15\tSystem info"));
    // echoln(F("M17\tAttach motors"));
    // echoln(F("M18\tDetach motors; same as M84"));
    // echoln(F("M70\tLaser status"));
    // echoln(F("M03\tLaser on"));
    // echoln(F("M05\tLaser off"));
    // echoln(F("M80\tPower on"));
    // echoln(F("M81\tPower off"));
    // echoln(F("M86\tAxes information"));
    // echoln(F("M87\tIs all done?"));
    // echoln(F("M88\tDistance measure"));
    // echoln(F("M89\tMemory information"));
    // echoln(F("M90\tFan information"));
    // echoln(F("M91\tTemperature information"));
    echoln(F("M92\tSystem information"));
    echoln(F("M99\tReset system"));
    // echoln(F("M100\tThis help message"));
    // echoln(F("M111\tDebug mode"));
    // echoln(F("M124\tStop all axes"));
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
// bool CommandM80() {
//  power.set(HIGH);
//  for (byte i=0; i<10; i++) {
//  delay(100);
//  if (!CommandM80a(false)) {  //  Power status
//  standby.reset();
//  return false;
// }
// }
// return true;
//}

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
// bool CommandM81() {
//  CommandM0();  //  Compulsory stop
//  CommandM72();  //  Laser off
//  standby_done = true;
//  standby_status = false;
//  standby.reset();
//  standby.disable();
//  power.set(LOW);
//  return power.status();
//}

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
void CommandM99() {
  echoln("Reseting...\n");
  b1.reset();
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
// bool CommandG28() {
//  if (!digitalRead(power_sensor_pin)) {
//  done = true;
//  status(true);
//  return true;
// }
// done = false;
// CommandG90();  //  Absolute programming
// x_axis.delayWrite(2);
// y_axis.delayWrite(2);
// x_axis.positionWrite(x_axis.parkRead());
// y_axis.positionWrite(y_axis.parkRead());
//}

/*
 *
 * Description
 *   Temperature information.
 *
 *   ()
 *
 * Parameters
 *   none
 *
 * Returns
 *   void
 */
// bool CommandM91() {
//  echoln(temperature.nameRead() + " (" +
//  temperature.status_name() + "): " +
//  temperature.valueRead() +
//  temperature.unitRead());
// if (debug_mode) {
// echoln("  Warning low: " +
// String(temperature.min_warningRead()) +
// temperature.unitRead() + "\n" +
// "  Critical low: " +
// String(temperature.min_criticalRead()) +
// temperature.unitRead() + "\n" +
// "  Warning high: " +
// String(temperature.max_warningRead()) +
// temperature.unitRead() + "\n" +
// "  Critical high: " +
// String(temperature.max_criticalRead()) +
// temperature.unitRead());
// }
//}

/* CommandM89
 *
 * Description
 *   Memory information.
 *
 *   ()
 *
 * Parameters
 *   none
 *
 * Returns
 *   void
 */
// bool CommandM89() {
// int total = 2 * 1024;
// int free = freeMemory();
// int used = total - free;
// int percent = (float)used * 100 / total;

// Alarm memory(75, 85);
// memory.nameWrite("Memory");
// memory.unitWrite("%");
// memory.check(percent);

// echoln(memory.nameRead() + " (" + memory.status_name() + "): " +
// percent + memory.unitRead() + " used");
// if (debug_mode) {
// echoln("  SRAM:\t" + String(total) + " B\n" +
//"  Used:\t" + used + " B\n" +
//"  Free:\t" + free + " B\n" +
//"  Warning: " + memory.max_warningRead() + memory.unitRead() + "\n" +
//"  Critical: " + memory.max_criticalRead() + memory.unitRead());
//}
//}

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
// void CommandM15() {
// CommandM92();  //  System information
// CommandM89();  //  Memory information
// CommandM80();  //  Power status
// CommandM91();  //  Temperature information
//}

/*
 *
 * Description
 *   System information.
 *
 *   ()
 *
 * Parameters
 *   none
 *
 * Returns
 *   void
 */
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

/* CommandM0
 *
 * Description
 *   Compulsory stop.
 *
 *   CommandM0()
 *
 * Parameters
 *   none
 *
 * Returns
 *   void
 */
// bool CommandM0() {
//  demonstration_period.set(0);
//  x_axis.positionWrite(x_axis.positionRead());
//  y_axis.positionWrite(y_axis.positionRead());
// return false;
//}

/* CommandM111
 *
 * Description
 *   Changes debug mode on or off.
 *
 *   CommandM111()
 *
 * Parameters
 *   none
 *
 * Returns
 *   void
 */
// void CommandM111() {
// debug_mode = !debug_mode;
// echoln("DEBUG: " + String(debug_mode ? F("on") : F("off")));
//}

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
// void Command0() {
// echoln(F("Unknown command"));
//}

/* CommandM18
 *
 * Description
 *   Detaches stepper motors.
 *
 *   CommandM18()
 *
 * Parameters
 *   none
 *
 * Returns
 *   void
 */
// void CommandM18() {
// CommandM0();  //  Compulsory stop
//  x_stepper.release();
//  y_stepper.release();
//}
