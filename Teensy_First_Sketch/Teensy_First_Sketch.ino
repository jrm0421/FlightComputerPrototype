#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "Adafruit_BMP3XX.h"
#include <math.h>
#include <SD.h>
#include <Kalman.h>

//PRESSURE IN HECTOPASCALS MUST BE REDEFINED BEFORE EACH FLIGHT
#define CURRENTPRESSURE_HPA (1013.25)
#define FILE_BASE_NAME "Data"

/*
   TO DO:
   WORK ON KALMAN FILTER FOR IMU
   ADD ABILITY TO VIEW ALT ABOVE GROUND RATHER THAN ABSOLUTE ALT
   SAVE DATA WITHOUT CRASHING TEENSY - DONE
   SAVE DATA TO FLASH, THEN TO SD ON LANDING
*/



const int RXPin = 0;
const int TXPin = 1;
const int SDPin = 4;
const uint16_t GPSBaud = 9600;

const int redPin = 10;
const int greenPin = 9;
const int bluePin = 8;
int redValue = 0;
int greenValue = 0;
int blueValue = 0;

unsigned long currentMillis = 0;
unsigned long previousFlashMillis = 0;
unsigned long previousLengthMillis = 0;
unsigned long previousSampleMillis = 0;

unsigned long currentLogMillis = 0;
unsigned long previousLogMillis = 0;
const int LogInterval = 2;

float alt;
float xRollRate;
float yRollRate;
float zRollRate;
float xCurrentAccel = 0;
float xPreviousVelocity = 0;
float xCurrentVelocity = 0;
float yCurrentAccel = 0;
float yPreviousVelocity = 0;
float yCurrentVelocity = 0;
float zCurrentAccel = 0;
float zPreviousVelocity = 0;
float zCurrentVelocity = 0;

uint32_t GPSDate;
uint32_t GPSTime;

const uint8_t BASE_NAME_SIZE = sizeof(FILE_BASE_NAME) - 1;
char fileName[] = FILE_BASE_NAME "00.csv";

TinyGPSPlus gps;
Adafruit_BMP3XX bmp;
Adafruit_BNO055 bno = Adafruit_BNO055(55);
File flightLogs;
SoftwareSerial gpsPort(RXPin, TXPin);

void setup() {
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(SDPin, OUTPUT);

  bno.setExtCrystalUse(true);
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  Wire.begin();
  gpsPort.begin(GPSBaud);
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  delay(1000);

  if (!SD.begin(SDPin)) {
    Serial.println("SD Card Reader Initialization Failed.");
    while (1) {
      currentMillis = millis();
      ledFlash(450, 50, 255, 0, 0);
      updateLeds();
    }
  }
  else {
    Serial.println("SD Card Reader Initialization Successful.");
  }

  delay(100);

  Serial.println("Testing SD Card Read/Write...");
  if (SD.exists("Initialization_Test.txt")) {
    Serial.println("Initialization_Test.txt already exists.");
    delay(100);
    Serial.println("Deleting Initialization_Test.txt...");
    SD.remove("Initialization_Test.txt");
    delay(100);
    if (SD.exists("Initialization_Test.txt")) {
      Serial.println("Failed to delete Initialization_Test.txt");
      while (1);
    } else {
      Serial.println("Initialization_Test.txt Successfully Deleted");
      delay(200);
    }
  }

  Serial.println("Attempting to create Initialization_Test.txt");
  flightLogs = SD.open("Initialization_Test.txt", FILE_WRITE);
  flightLogs.println("This is a test of the SD card reader.");
  flightLogs.close();
  delay(100);
  if (SD.exists("Initialization_Test.txt")) {
    Serial.println("Initialization_Test.txt created successfully.");
    Serial.println("Reading its contents...");
    flightLogs = SD.open("Initialization_Test.txt");
    delay(100);
    Serial.print("The contents of the file are: ");
    while (flightLogs.available()) {
      Serial.write(flightLogs.read());
    }
    flightLogs.close();
    Serial.println();
  } else {
    Serial.println("Initialization_Test.txt does not exist.");
  }

  delay(1000);

  while (SD.exists(fileName)) {
    if (fileName[BASE_NAME_SIZE + 1] != '9') {
      fileName[BASE_NAME_SIZE + 1]++;
    } else if (fileName[BASE_NAME_SIZE] != '9') {
      fileName[BASE_NAME_SIZE + 1] = '0';
      fileName[BASE_NAME_SIZE]++;
    } else {
      Serial.println(F("Unable to create new file name"));
      ledFlash(100, 50, 255, 0, 0);
      while (1);
    }
  }

  delay(1000);
  if (!bno.begin()) {
    Serial.println("BNO055 Initialization Failed.");
    while (1) {
      currentMillis = millis();
      ledFlash(100, 50, 255, 0, 0);
      updateLeds();
    }
  } else {
    delay(100);
    Serial.println("BNO055 Initialization Successful");
  }

  if (!bmp.begin_I2C()) {
    Serial.println("BMP380 Initialization Failed.");
    while (1) {
      currentMillis = millis();
      ledFlash(250, 50, 255, 0, 0);
      updateLeds();
    }
  } else {
    delay(100);
    Serial.println("BMP380 Initialization Successful");
  }
  delay(500);

  Serial.println("Testing LED light...");
  redValue = 255; greenValue = 255; blueValue = 255;
  updateLeds();
  delay(2000);
  redValue = 0; greenValue = 0; blueValue = 0;
  updateLeds();
  Serial.println("LED Testing Complete.");

  delay(2000);
}

void loop() {
  currentMillis = millis();
  //checkState();
  updateLeds();
  getOrientation();
  getAccelAndAlt();
  readRollRate();
  accelToVel();
  fileWrite();
  smartDelay(10);
  printGPS();
  Serial.println();
  Serial.println();
}

void fileWrite() {
  currentLogMillis = millis();
  if ((currentLogMillis - previousLogMillis) >= LogInterval) {
    flightLogs = SD.open(fileName, FILE_WRITE);
    if (flightLogs) {
      sensors_event_t event;
      bno.getEvent(&event);
      imu::Vector<3> lineacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

      flightLogs.print("\tms: "); flightLogs.print(millis());
      flightLogs.print("\tAccX: "); flightLogs.print(lineacc.x());
      flightLogs.print("\tAccY: "); flightLogs.print(lineacc.y());
      flightLogs.print("\tAccZ: "); flightLogs.print(lineacc.z());
      flightLogs.print("\tOrientX: "); flightLogs.print(event.orientation.x);
      flightLogs.print("\tOrientY: "); flightLogs.print(event.orientation.y);
      flightLogs.print("\tOrientZ: "); flightLogs.print(event.orientation.z);
      flightLogs.print("\txCurVel: "); flightLogs.print(xCurrentVelocity);
      flightLogs.print("\tyCurVel: "); flightLogs.print(yCurrentVelocity);
      flightLogs.print("\tzCurVel: "); flightLogs.print(zCurrentVelocity);
      flightLogs.print("\txRollRate: "); flightLogs.print(xRollRate);
      flightLogs.print("\tyRollRate: "); flightLogs.print(yRollRate);
      flightLogs.print("\tzRollRate: "); flightLogs.print(zRollRate);
      flightLogs.print("\tBaroAlt: "); flightLogs.print(alt);
      flightLogs.print("\tGPSLat: "); flightLogs.print(gps.location.lat(), 6);
      flightLogs.print("\tGPSLng: "); flightLogs.print(gps.location.lng(), 6);
      flightLogs.print("\tGPSDate: "); flightLogs.print(gps.date.value());
      flightLogs.print("\tGPSTime: "); flightLogs.print(gps.time.value());
      flightLogs.print("\tGPSAlt: "); flightLogs.print(gps.altitude.meters());
      flightLogs.print("\tGPSSpeedMPS: "); flightLogs.print(gps.speed.mps());
      flightLogs.print("\tSats: "); flightLogs.println(gps.satellites.value());

      flightLogs.close();

      previousLogMillis = millis();
    }
  }
}

void getOrientation() {
  sensors_event_t event;
  bno.getEvent(&event);
  Serial.print("OrientX: "); Serial.print(event.orientation.x);
  Serial.print("\tOrientY: "); Serial.print(event.orientation.y);
  Serial.print("\tOrientZ: "); Serial.print(event.orientation.z);
}

void getAccelAndAlt() {
  sensors_event_t event;
  bno.getEvent(&event);
  imu::Vector<3> lineacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  Serial.print("\tAccX: "); Serial.print(lineacc.x());
  Serial.print("\tAccY: "); Serial.print(lineacc.y());
  Serial.print("\tAccZ: "); Serial.print(lineacc.z());
  alt = bmp.readAltitude(CURRENTPRESSURE_HPA);
  Serial.print("\tAlt: "); Serial.println(alt);
}

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (gpsPort.available())
      gps.encode(gpsPort.read());
  } while (millis() - start < ms);
}

void printGPS() {
  if (gps.location.isValid()) {
    ledFlash(3000, 20, 0, 255, 0);
    Serial.print("Latitude: ");
    Serial.print(gps.location.lat(), 6);

    Serial.print("\tLongitude: ");
    Serial.print(gps.location.lng(), 6);

  } else {
    ledFlash(2000, 100, 255, 255, 0);
    Serial.print("Latitude: ");
    Serial.print("*******");
    Serial.print("\tLongitude: ");
    Serial.print("*******");
  }

  Serial.print("\tDate: ");
  if (gps.date.isValid()) {
    GPSDate = gps.date.value();
    Serial.print(gps.date.value());
  } else {
    Serial.print("*******");
  }

  Serial.print("\tTime: ");
  if (gps.time.isValid()) {
    GPSTime = gps.time.value();
    Serial.print(gps.time.value());
  } else {
    Serial.print("*******");
  }

  Serial.print("\tSats: ");
  if (gps.satellites.isValid()) {
    Serial.print(gps.satellites.value());
  } else {
    Serial.print("*******");
  }
}

void ledFlash(int flashInterval, int flashLength, int r, int g, int b) {
  if ((redValue == 0) && (greenValue == 0) && (blueValue == 0)) {
    if (currentMillis - previousFlashMillis >= flashInterval) {
      redValue = r;
      greenValue = g;
      blueValue = b;
      previousFlashMillis += flashInterval;
    }
  } else {
    if (currentMillis - previousFlashMillis >= flashLength) {
      redValue = 0;
      greenValue = 0;
      blueValue = 0;
      previousLengthMillis += flashLength;
    }
  }
}

void updateLeds() {
  analogWrite(redPin, redValue);
  analogWrite(greenPin, greenValue);
  analogWrite(bluePin, blueValue);
}

void accelToVel() {
  //NEEDS KALMAN FILTER BADLY
  //This is entirely experimental. DO NOT TRUST

  if (currentMillis - previousSampleMillis >= 10) {
    imu::Vector<3> lineacc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    xCurrentAccel = (lineacc.x()) * (currentMillis - previousSampleMillis);
    yCurrentAccel = (lineacc.y()) * (currentMillis - previousSampleMillis);
    zCurrentAccel = (lineacc.z()) * (currentMillis - previousSampleMillis);
    xPreviousVelocity = xCurrentAccel;
    yPreviousVelocity = yCurrentAccel;
    zPreviousVelocity = zCurrentAccel;
    xCurrentVelocity += xPreviousVelocity;
    yCurrentVelocity += yPreviousVelocity;
    zCurrentVelocity += zPreviousVelocity;
    previousSampleMillis = millis();
    Serial.print("\txCurrentVelocity: "); Serial.print(xCurrentVelocity);
    Serial.print("\tyCurrentVelocity: "); Serial.print(yCurrentVelocity);
    Serial.print("\tzCurrentVelocity: "); Serial.println(zCurrentVelocity);
  }
}

void readRollRate() {
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  Serial.print("X Roll Rate: "); xRollRate = ((180 / 3.14159265359) * (gyroscope.x())); Serial.print(xRollRate); Serial.print(" deg/s");
  Serial.print("\tY Roll Rate: "); yRollRate = ((180 / 3.14159265359) * (gyroscope.y())); Serial.print(yRollRate); Serial.print(" deg/s");
  Serial.print("\tZ Roll Rate: "); zRollRate = ((180 / 3.14159265359) * (gyroscope.z())); Serial.print(zRollRate); Serial.print(" deg/s");
}

void checkState() {
  //list of states: startup, attempting gps lock, gps locked, calibrating sensors, ready for launch/on pad, launched, burnout, coast, apex, unpowered descent, chutes, under chutes, touchdown
  //make sure to change data logging rate to avoid using up all the space lol
  ;
}
