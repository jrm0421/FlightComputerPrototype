#include <globalDeclarations.h>
#include <libraries.h>

// Including other header files leads to MANY multiple definition errors... *sigh*
// C++ formatting is hard to learn... >:(
// At least it compiles ¯\_(ツ)_/¯

// Looking back on this file structure makes me realize how much I overcomplicated it...

/*
   TO DO:
   WORK ON KALMAN FILTER FOR IMU
   ADD ABILITY TO VIEW ALT ABOVE GROUND RATHER THAN ABSOLUTE ALT
   SAVE DATA TO FLASH, THEN TO SD ON LANDING
   STATE DETECTION
*/

void setup() {
  clearArrays(60);
  definePinModes();
  configureSensors();
  communicationsBegin();
  initializeSD();
  initializeBNO();
  initializeBMP();
  initializeLED();
  delay(2000);
}

void loop() {
  currentMillis = millis();
  checkState();
  updateLeds();
  getOrientation();
  imuCalibration();
  getAccelAndAlt();
  getAltAboveGround();
  readRollRate();
  accelToVel();
  fileWrite();
  printHomeData();
  smartDelay(10);
  printGPS();
  Serial.println();
  Serial.println();
}