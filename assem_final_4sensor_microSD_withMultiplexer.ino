/*
  full verson of multiplex 4 sensor to SD card and print to serial monitor ***final and can compile. 

  B U T !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

  -> problem: sketch too big exceede 32 kbytes for the arduino uno.

*/


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SPI.h>
#include <SD.h>

// === Multiplexer ===
void tca_select(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(0x70);
  Wire.write(1 << channel);
  Wire.endTransmission();
} 

// === BME280 ===
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;
bool bme_ok = false;

// === MPU6050 Kalman ===
float RateRoll1, RatePitch1, RateYaw1;
float RateRoll2, RatePitch2, RateYaw2;
float RateCalibrationRoll1 = 0, RateCalibrationPitch1 = 0, RateCalibrationYaw1 = 0;
float RateCalibrationRoll2 = 0, RateCalibrationPitch2 = 0, RateCalibrationYaw2 = 0;
int RateCalibrationNumber;

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float KalmanAngleRoll1 = 0, KalmanUncertaintyAngleRoll1 = 4;
float KalmanAnglePitch1 = 0, KalmanUncertaintyAnglePitch1 = 4;
float KalmanAngleRoll2 = 0, KalmanUncertaintyAngleRoll2 = 4;
float KalmanAnglePitch2 = 0, KalmanUncertaintyAnglePitch2 = 4;
float Kalman1DOutput[] = {0, 0};
uint32_t LoopTimer;
bool mpu1_ok = true, mpu2_ok = true;

// === GPS ===
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// === SD ===
#define SD_CS 10
File logfile;

// === Kalman Filter ===
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState += 0.004 * KalmanInput;
  KalmanUncertainty += 0.004 * 0.004 * 16;
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 9);
  KalmanState += KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty *= (1 - KalmanGain);
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

bool gyro_signals(uint8_t channel, uint8_t address, float &RateRoll, float &RatePitch, float &RateYaw) {
  tca_select(channel);
  Wire.beginTransmission(address);
  Wire.write(0x3B);
  if (Wire.endTransmission() != 0) return false;
  Wire.requestFrom(address, 6);
  if (Wire.available() < 6) return false;
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(address);
  Wire.write(0x43);
  if (Wire.endTransmission() != 0) return false;
  Wire.requestFrom(address, 6);
  if (Wire.available() < 6) return false;
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;

  AccX = (float)AccXLSB / 4096 - 0.05;
  AccY = (float)AccYLSB / 4096 + 0.01;
  AccZ = (float)AccZLSB / 4096 - 0.11;

  AngleRoll = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * 180 / PI;
  AnglePitch = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * 180 / PI;

  return true;
}

void setup() {
  Serial.begin(57600);
  Wire.setClock(400000);
  Wire.begin();
  ss.begin(GPSBaud);

  // === SD Card ===
  if (!SD.begin(SD_CS)) {
    Serial.println("SD fail");
  } else {
    logfile = SD.open("IRECdataloging.txt", FILE_WRITE);
    if (logfile) {
      logfile.println("Timestamp,Humidity,Temp,Pressure,Alt,Roll1,Pitch1,Roll2,Pitch2,Lat,Lon,Speed,GPS_Alt"); // tell what variable will be log in the sd
      logfile.close();
    }
  }

  // === BME280 ===
  tca_select(1);
  bme_ok = bme.begin(0x76);

  // === MPU6050 #1 (channel 6, addr 0x68) ===
  tca_select(6);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    if (!gyro_signals(6, 0x68, RateRoll1, RatePitch1, RateYaw1)) {
      mpu1_ok = false; break;
    }
    RateCalibrationRoll1 += RateRoll1;
    RateCalibrationPitch1 += RatePitch1;
    RateCalibrationYaw1 += RateYaw1;
    delay(1);
  }
  if (mpu1_ok) {
    RateCalibrationRoll1 /= 2000;
    RateCalibrationPitch1 /= 2000;
    RateCalibrationYaw1 /= 2000;
  }

  // === MPU6050 #2 (channel 7, addr 0x69) ===
  tca_select(7);
  Wire.beginTransmission(0x69);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    if (!gyro_signals(7, 0x69, RateRoll2, RatePitch2, RateYaw2)) {
      mpu2_ok = false; break;
    }
    RateCalibrationRoll2 += RateRoll2;
    RateCalibrationPitch2 += RatePitch2;
    RateCalibrationYaw2 += RateYaw2;
    delay(1);
  }
  if (mpu2_ok) {
    RateCalibrationRoll2 /= 2000;
    RateCalibrationPitch2 /= 2000;
    RateCalibrationYaw2 /= 2000;
  }

  LoopTimer = micros();
}

void loop() {
  while (ss.available()) gps.encode(ss.read());

  float hum = -1, temp = -1, pres = -1, alt = -1;
  float roll1 = -1, pitch1 = -1;
  float roll2 = -1, pitch2 = -1;

  // === BME280 ===
  tca_select(1);
  if (bme_ok) {
    hum = bme.readHumidity();
    temp = bme.readTemperature();
    pres = bme.readPressure() / 100.0F;
    alt = bme.readAltitude(SEALEVELPRESSURE_HPA);
  }

  // === MPU6050 #1 ===
  if (mpu1_ok && gyro_signals(6, 0x68, RateRoll1, RatePitch1, RateYaw1)) {
    RateRoll1 -= RateCalibrationRoll1;
    RatePitch1 -= RateCalibrationPitch1;
    kalman_1d(KalmanAngleRoll1, KalmanUncertaintyAngleRoll1, RateRoll1, AngleRoll);
    KalmanAngleRoll1 = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll1 = Kalman1DOutput[1];
    kalman_1d(KalmanAnglePitch1, KalmanUncertaintyAnglePitch1, RatePitch1, AnglePitch);
    KalmanAnglePitch1 = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch1 = Kalman1DOutput[1];
    roll1 = KalmanAngleRoll1;
    pitch1 = KalmanAnglePitch1;
  }

  // === MPU6050 #2 ===
  if (mpu2_ok && gyro_signals(7, 0x69, RateRoll2, RatePitch2, RateYaw2)) {
    RateRoll2 -= RateCalibrationRoll2;
    RatePitch2 -= RateCalibrationPitch2;
    kalman_1d(KalmanAngleRoll2, KalmanUncertaintyAngleRoll2, RateRoll2, AngleRoll);
    KalmanAngleRoll2 = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll2 = Kalman1DOutput[1];
    kalman_1d(KalmanAnglePitch2, KalmanUncertaintyAnglePitch2, RatePitch2, AnglePitch);
    KalmanAnglePitch2 = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch2 = Kalman1DOutput[1];
    roll2 = KalmanAngleRoll2;
    pitch2 = KalmanAnglePitch2;
  }

  // === GPS ===
  double lat = gps.location.isValid() ? gps.location.lat() : -1;
  double lon = gps.location.isValid() ? gps.location.lng() : -1;
  double speed = gps.speed.isValid() ? gps.speed.mps() : -1;
  double gps_alt = gps.altitude.isValid() ? gps.altitude.meters() : -1;

  // === Time ===
  String timestamp = "";
  if (gps.time.isValid() && gps.date.isValid()) {
    char buffer[25];
    sprintf(buffer, "%04d-%02d-%02d %02d:%02d:%02d",
            gps.date.year(), gps.date.month(), gps.date.day(),
            gps.time.hour(), gps.time.minute(), gps.time.second());
    timestamp = String(buffer);
  } else {
    timestamp = "millis_" + String(millis());
  }

  // === Data Row ===
  String dataRow = timestamp + "," +
                   String(hum) + "," + String(temp) + "," + String(pres) + "," + String(alt) + "," +
                   String(roll1) + "," + String(pitch1) + "," +
                   String(roll2) + "," + String(pitch2) + "," +
                   String(lat, 6) + "," + String(lon, 6) + "," + String(speed) + "," + String(gps_alt);

  // === Print to Serial ===
  Serial.println(dataRow);

  // === Write to SD ===
  logfile = SD.open("IRECdataloging.txt", FILE_WRITE);
  if (logfile) {
    logfile.println(dataRow);
    logfile.close();
  }

  while (micros() - LoopTimer < 4000);  // maintain loop timing ~250Hz
  LoopTimer = micros();
}
