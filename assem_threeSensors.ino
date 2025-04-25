/*
 This IREC code test to combine data from BME280, MPU6050 (address 0x68), and GPS (TinyGPS++)
 With robustness against sensor failures.
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// === BME280 Setup ===
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;
bool bme_ok = false;

// === MPU6050 Kalman Filter Setup ===
float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
uint32_t LoopTimer;
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};
bool mpu_ok = true;

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + 0.004 * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3);
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

bool gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  if (Wire.endTransmission() != 0) return false; // sensor not responding
  Wire.requestFrom(0x68, 6);
  if (Wire.available() < 6) return false;
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  if (Wire.endTransmission() != 0) return false;
  Wire.requestFrom(0x68, 6);
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

  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 180 / PI;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 180 / PI;

  return true;
}

// === GPS Setup ===
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(57600);
  Wire.setClock(400000);
  Wire.begin();

  // BME280 setup
  bme_ok = bme.begin(0x76);

  // MPU6050 setup
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Power Management
  Wire.write(0x00);
  Wire.endTransmission();
  delay(250);

  // Calibration
  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    if (!gyro_signals()) {
      mpu_ok = false;
      break;
    }
    RateCalibrationRoll += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw += RateYaw;
    delay(1);
  }

  if (mpu_ok) {
    RateCalibrationRoll /= 2000;
    RateCalibrationPitch /= 2000;
    RateCalibrationYaw /= 2000;
  }

  ss.begin(GPSBaud);
  LoopTimer = micros();
}

void loop() {
  // GPS Read
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

  // Read MPU if available
  bool mpu_read_ok = false;
  if (mpu_ok) {
    mpu_read_ok = gyro_signals();
    if (mpu_read_ok) {
      RateRoll -= RateCalibrationRoll;
      RatePitch -= RateCalibrationPitch;
      RateYaw -= RateCalibrationYaw;

      kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
      KalmanAngleRoll = Kalman1DOutput[0];
      KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

      kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
      KalmanAnglePitch = Kalman1DOutput[0];
      KalmanUncertaintyAnglePitch = Kalman1DOutput[1];
    }
  }

  // === Output ===
  // the -- will output in the value if the sensor is malfunction. this make the program robust (if one fail, it will still report others and not stuck to .begin (hope so))
  Serial.print(bme_ok ? String(bme.readHumidity()) : "--"); Serial.print("% | ");
  Serial.print(bme_ok ? String(bme.readTemperature()) : "--"); Serial.print("*C | ");
  Serial.print(bme_ok ? String(bme.readPressure() / 100.0F) : "--"); Serial.print("hPa | ");
  Serial.print(bme_ok ? String(bme.readAltitude(SEALEVELPRESSURE_HPA)) : "--"); Serial.print("m | ");

  Serial.print(mpu_read_ok ? String(KalmanAngleRoll) : "--"); Serial.print("° | ");
  Serial.print(mpu_read_ok ? String(KalmanAnglePitch) : "--"); Serial.print("° | ");

  if (gps.location.isUpdated()) {
    Serial.print("Lat: "); Serial.print(gps.location.lat(), 6); Serial.print(" | ");
    Serial.print("Lon: "); Serial.print(gps.location.lng(), 6); Serial.print(" | ");
    Serial.print("Speed: "); Serial.print(gps.speed.mps()); Serial.print(" m/s | ");
    Serial.print("Alt: "); Serial.print(gps.altitude.meters()); Serial.println(" m");
  } else {
    Serial.println("GPS: --");
  }

  while (micros() - LoopTimer < 10000);
  LoopTimer = micros();
}
