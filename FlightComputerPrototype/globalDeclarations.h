#ifndef GLOBALDECLARATIONS_H
#define GLOBALDECLARATIONS_H

//TinyGPSPlus
extern const int RXPin;
extern const int TXPin;
extern const uint16_t GPSBaud;

//LED Pins
extern const int redPin;
extern const int greenPin;
extern const int bluePin;
extern int redValue;
extern int greenValue;
extern int blueValue;

//Timings for LED flashes
extern unsigned long currentMillis;
extern unsigned long previousFlashMillis;
extern unsigned long previousLengthMillis;

//Last acceleration sample time
extern unsigned long previousSampleMillis;

//Timings for SD card data logging
extern unsigned long currentLogMillis;
extern unsigned long previousLogMillis;
extern const unsigned int logInterval;

//IMU calibration variables
extern uint8_t bnoSystem;
extern uint8_t gyro;
extern uint8_t accel;
extern uint8_t mg;

//Attitude variables
#define CURRENTPRESSURE_HPA (1013.25)   //Current sea level air pressure in hPa. Must redefine each flight.
extern float alt;
extern float xRollRate;
extern float yRollRate;
extern float zRollRate;
extern float xCurrentAccel;
extern float xPreviousVelocity;
extern float xCurrentVelocity;
extern float yCurrentAccel;
extern float yPreviousVelocity;
extern float yCurrentVelocity;
extern float zCurrentAccel;
extern float zPreviousVelocity;
extern float zCurrentVelocity;

//SD card file naming
extern const int SDPin;
#define FILE_BASE_NAME "Data"           //Base name for log file on SD card
extern const uint8_t BASE_NAME_SIZE;
extern char fileName[];

//Object declarations
extern TinyGPSPlus gps;
extern Adafruit_BMP3XX bmp;
extern Adafruit_BNO055 bno;
extern File flightLogs;
extern SoftwareSerial gpsPort;

//Flight states. Only one can be true at a time
extern bool booting;
extern bool bootSuccess;
extern bool calibrating;
extern bool calibrationSuccess;
extern bool gpsLocking;
extern bool gpsLockingSuccess;
extern bool onPad;
extern bool readyForLaunch;
extern bool poweredAscent;
extern bool coasting;
extern bool pastApogee;
extern bool unpoweredDescent;
extern bool chuteDeploy;
extern bool underChutes;
extern bool touchdown;
extern bool vehicleSafe;

//checkOnPad variables
extern unsigned long previousZeroMillis;
extern double longitudeValues[60];
extern double latitudeValues[60];
extern float altitudeValues[60];
extern double latMax;
extern double lngMax;
extern double latMin;
extern double lngMin;
extern double latRange;
extern double lngRange;
extern float altitudeAverage;
extern float altitudeSum;
extern float altAboveGround;
extern int checkOnPadIndex;
extern int averageTimes;
extern bool stopFillingArray;
extern bool lateralMovement;
extern bool verticalMovement;

extern double homeLat;
extern double homeLng;
extern float homeAlt;

extern void ledFlash();
extern void updateLeds();
extern void communicationsBegin();
extern void definePinModes();
extern void configureSensors();
extern void initializeSD();
extern void initializeBNO();
extern void initializeBMP();
extern void initializeLED();
extern void getOrientation();
extern void imuCalibration();
extern void getAccelAndAlt();
extern void getAltAboveGround();
extern void readRollRate();
extern void accelToVel();
extern void fileWrite();
extern void smartDelay(unsigned long ms);
extern void printGPS();
extern void clearArrays(int arraySize);
extern void checkState();
extern void zeroValues();
extern void checkOnPad();
extern void printHomeData();

#endif