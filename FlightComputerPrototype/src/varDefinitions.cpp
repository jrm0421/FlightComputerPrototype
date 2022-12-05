#ifndef VARDEFINITIONS_CPP
#define VARDEFINITIONS_CPP

#include <globalDeclarations.h>
#include <libraries.h>

//TinyGPSPlus
const int RXPin = 0;
const int TXPin = 1;
const uint16_t GPSBaud = 9600;

//LED Pins
const int redPin = 10;
const int greenPin = 9;
const int bluePin = 8;
int redValue = 0;
int greenValue = 0;
int blueValue = 0;

#define CURRENTPRESSURE_HPA (1013.25)   //Current sea level air pressure in hPa. Must redefine each flight.

float alt = 0;
float xRollRate = 0;
float yRollRate = 0;
float zRollRate = 0;

//Timings for LED flashes
unsigned long currentMillis = 0;
unsigned long previousFlashMillis = 0;
unsigned long previousLengthMillis = 0;

//Last acceleration sample time
unsigned long previousSampleMillis = 0;

//Timings for SD card data logging
unsigned long currentLogMillis = 0;
unsigned long previousLogMillis = 0;
unsigned const int logInterval = 2;

//IMU calibration variables
uint8_t bnoSystem = 0;
uint8_t gyro = 0;
uint8_t accel = 0;
uint8_t mg = 0;

TinyGPSPlus gps;
Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno;
File flightLogs;
SoftwareSerial gpsPort(RXPin, TXPin);

//SDCard Stuff
const int SDPin = 4;
#define FILE_BASE_NAME "Data"
char fileName[] = FILE_BASE_NAME "00.csv";
const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;

//IMU Variables
float xCurrentAccel = 0;
float xPreviousVelocity = 0;
float xCurrentVelocity = 0;
float yCurrentAccel = 0;
float yPreviousVelocity = 0;
float yCurrentVelocity = 0;
float zCurrentAccel = 0;
float zPreviousVelocity = 0;
float zCurrentVelocity = 0;

unsigned long previousZeroMillis = 0;
double longitudeValues[60];
double latitudeValues[60];
float altitudeValues[60];
double latMax = latitudeValues[0];
double lngMax = longitudeValues[0];
double latMin = latitudeValues[0];
double lngMin = longitudeValues[0];
double latRange = 0;
double lngRange = 0;
float altitudeAverage = 0;
float altitudeSum = 0;
float altAboveGround = 0;
int checkOnPadIndex = 0;
int averageTimes = 0;
bool stopFillingArray = false;
bool lateralMovement = true;
bool verticalMovement = true;
double homeLat = 0;
double homeLng = 0;
float homeAlt = 0;

//Flight states. Only one can be true at a time
bool booting = true;
bool bootSuccess = false;
bool calibrating = false;
bool calibrationSuccess = false;
bool gpsLocking = false;
bool gpsLockingSuccess = false;
bool onPad = false;
bool readyForLaunch = false;
bool poweredAscent = false;
bool coasting = false;
bool pastApogee = false;
bool unpoweredDescent = false;
bool chuteDeploy = false;
bool underChutes = false;
bool touchdown = false;
bool vehicleSafe = false;

#endif