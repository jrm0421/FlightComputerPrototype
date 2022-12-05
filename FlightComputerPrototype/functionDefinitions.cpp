#ifndef FUNCTIONS_CPP
#define FUNCTIONS_CPP

#include <globalDeclarations.h>
#include <libraries.h>

void ledFlash(unsigned int flashInterval, unsigned int flashLength, int r, int g, int b) {
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

void communicationsBegin() {
  Wire.begin();
  gpsPort.begin(GPSBaud);
  Serial.begin(115200);
  while (!Serial) {;};
}

void definePinModes() {
    pinMode(redPin, OUTPUT);
    pinMode(greenPin, OUTPUT);
    pinMode(bluePin, OUTPUT);
    pinMode(SDPin, OUTPUT);
}

void configureSensors() {
    bno.setExtCrystalUse(true);
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
}

void initializeSD() {
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

}

void initializeBNO() {
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
}

void initializeBMP() {
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
}

void initializeLED() {
    Serial.println("Testing LED light...");
    redValue = 255; greenValue = 255; blueValue = 255;
    updateLeds();
    delay(2000);
    redValue = 0; greenValue = 0; blueValue = 0;
    updateLeds();
    Serial.println("LED Testing Complete.");

}

void getOrientation() {
    sensors_event_t event;
    bno.getEvent(&event);
    Serial.print("OrientX: "); Serial.print(event.orientation.x);
    Serial.print("\tOrientY: "); Serial.print(event.orientation.y);
    Serial.print("\tOrientZ: "); Serial.print(event.orientation.z);
}

void imuCalibration() {
  bno.getCalibration(&bnoSystem, &gyro, &accel, &mg);
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

void getAltAboveGround() {
  altAboveGround = alt - altitudeAverage;
  Serial.println("\tAltAboveGnd: "); Serial.println(altAboveGround);
}

void readRollRate() {
    imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    Serial.print("X Roll Rate: "); xRollRate = ((180 / 3.14159265359) * (gyroscope.x())); Serial.print(xRollRate); Serial.print(" deg/s");
    Serial.print("\tY Roll Rate: "); yRollRate = ((180 / 3.14159265359) * (gyroscope.y())); Serial.print(yRollRate); Serial.print(" deg/s");
    Serial.print("\tZ Roll Rate: "); zRollRate = ((180 / 3.14159265359) * (gyroscope.z())); Serial.print(zRollRate); Serial.print(" deg/s");
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

void fileWrite() {
  currentLogMillis = millis();
  if ((currentLogMillis - previousLogMillis) >= logInterval) {
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

void smartDelay(unsigned long ms) {
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
    Serial.print(gps.date.value());
  } else {
    Serial.print("*******");
  }

  Serial.print("\tTime: ");
  if (gps.time.isValid()) {
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



void clearArrays(int arraySize) {
  for(int i = 0; i < arraySize; ++i) {
    altitudeValues[i] = 0;
    latitudeValues[i] = 0;
    longitudeValues[i] = 0;
  }
}

void checkState() {
    for(int i = 0; i == 1; i++) {   //Only runs the first time through the loop
        if((SD.begin(SDPin)) && (bno.begin()) && (bmp.begin_I2C())) {   //If all components began successfully, assume successful boot.
          booting = false;
          bootSuccess = true;  
        }
    }

    if(bootSuccess) {
        if((bnoSystem <= 1) || (gyro <= 1) || (accel <= 1) || (mg <= 1)) { //If booted successfully and imu calibration values 0-1 (out of 3), assume calibration incomplete.
            calibrationSuccess = false;
            calibrating = true;
        }
        else {
            calibrating = false;
            calibrationSuccess = true;
        }
    }

    if((calibrationSuccess) && (!gps.location.isValid())) { //If sensors are calibrated and GPS location is invalid, assume locking incomplete.
        gpsLockingSuccess = false;
        gpsLocking = true;
    } else {
        gpsLocking = false;
        gpsLockingSuccess = true;
    }

    if(bootSuccess && calibrationSuccess && gpsLockingSuccess && !lateralMovement && !verticalMovement) { //If all of the above are true, ready for launch.
        readyForLaunch = true;
        double homeLat = (gps.location.lat(), 6);
        double homeLng = (gps.location.lng(), 6);
        float homeAlt = alt;
        //Maybe change these so that the constant values are defined EXACTLY at ignition...
    } else {
        //checkOnPad();
    }
}

void printHomeData() {
  Serial.print("HomeLat: "); Serial.print(homeLat);
  Serial.print("\tHomeLng: "); Serial.print(homeLng);
  Serial.print("\tHomeAlt: "); Serial.println(homeAlt);
}

void zeroValues() {
    /* If vehicle is on pad, zero out certain values for reference.
    On pad state determined:
        vehicle is not moving for more than one minute (based on GPS)
        vehicle has very low acceleration
        vehicle has unchanging altitude
        vehicle has temperature within acceptable outdoor range
        vehicle has lock on gps, imu calibration, and booted successfully
        vehicle is oriented upwards
    maybe have physical overide in the form of a button? will figure this out in the future
    */
}

void checkOnPad() {
    if((currentMillis - previousZeroMillis >= 1000) && (stopFillingArray == false)) { //if one second has passed and no one told it to stop, do this once per loop
        do {
            altitudeValues[checkOnPadIndex] = bmp.readAltitude(CURRENTPRESSURE_HPA);
            latitudeValues[checkOnPadIndex] = (gps.location.lat(), 6);
            longitudeValues[checkOnPadIndex] = (gps.location.lng(), 6);
            checkOnPadIndex++;
        } while ((altitudeValues[59] == 0) && (longitudeValues[59] == 0) && (latitudeValues[59] == 0));
        previousZeroMillis = millis();
   }


   if((latitudeValues[59] != 0) && (longitudeValues[59] != 0) && (altitudeValues[59] != 0) && (averageTimes == 0)) {
    stopFillingArray = true;

    for(unsigned int minMaxIndex = 0; minMaxIndex < (sizeof(latitudeValues)/sizeof(latitudeValues[0])); minMaxIndex++) {
        latMax = max(latitudeValues[minMaxIndex], latMax);
        latMin = min(latitudeValues[minMaxIndex], latMin);
        latRange = latMax - latMin;
    }
    for(unsigned int minMaxIndex = 0; minMaxIndex < (sizeof(longitudeValues)/sizeof(longitudeValues[0])); minMaxIndex++) {
        lngMax = max(longitudeValues[minMaxIndex], lngMax);
        lngMin = min(longitudeValues[minMaxIndex], lngMin);
        lngRange = lngMax - lngMin;
    }
    for(int averageIndex = 0; averageIndex < 60; averageIndex++) {
        altitudeSum += altitudeValues[averageIndex];
    }
    altitudeAverage = altitudeSum / 60;
    averageTimes = 1;
    }

    if((latRange <= 1) && (lngRange <= 1)) {
        lateralMovement = false;
    } else {
        lateralMovement = true;
    }
}

#endif