/*
  this code collect the gps data + mpu6050 1 & 2
  GPS: 
    TX <-> pin 4
    RX <-> pin 3
    VCC <-> 5V
    GND <-> 5V
  
  MPU6050 1#:
    VCC and GND as it should
    SDA <-> A4
    SCL <-> A5
    AD0 <-> GND (adress I2C 0x68)

  MPU6050 2#:
    VCC and GND as it should
    SDA <-> A4
    SCL <-> A5
    AD0 <-> 5V (adress I2C 0x69)

*/




#include <Wire.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// ======== GPS SETUP ========
static const int RXPin = 4, TXPin = 3;  // GPS Module
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// ======== MPU6050 Variables ========
float RateCalibrationRoll1 = 0, RateCalibrationPitch1 = 0, RateCalibrationYaw1 = 0;
float RateCalibrationRoll2 = 0, RateCalibrationPitch2 = 0, RateCalibrationYaw2 = 0;

float RateRoll1 = 0, RatePitch1 = 0, RateYaw1 = 0;
float RateRoll2 = 0, RatePitch2 = 0, RateYaw2 = 0;

float AccX1 = 0, AccY1 = 0, AccZ1 = 0;
float AccX2 = 0, AccY2 = 0, AccZ2 = 0;

float AngleRoll1 = 0, AnglePitch1 = 0;
float AngleRoll2 = 0, AnglePitch2 = 0;

float KalmanAngleRoll1 = 0, KalmanUncertaintyAngleRoll1 = 4;
float KalmanAnglePitch1 = 0, KalmanUncertaintyAnglePitch1 = 4;
float KalmanAngleYaw1 = 0, KalmanUncertaintyAngleYaw1 = 4;

float KalmanAngleRoll2 = 0, KalmanUncertaintyAngleRoll2 = 4;
float KalmanAnglePitch2 = 0, KalmanUncertaintyAnglePitch2 = 4;
float KalmanAngleYaw2 = 0, KalmanUncertaintyAngleYaw2 = 4;

float dt = 0.004; // 4ms
uint32_t LoopTimer = 0;

// ======== FUNCTIONS ========
void kalman_1d(float& KalmanState, float& KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + dt * KalmanInput;
  KalmanUncertainty += dt * dt * 16;
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 9);
  KalmanState += KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty *= (1 - KalmanGain);
}

void gyro_signals(uint8_t address) {
  // Configure MPU
  Wire.beginTransmission(address);
  Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(0x3B); Wire.endTransmission();
  Wire.requestFrom(address, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(address);
  Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
  Wire.beginTransmission(address);
  Wire.write(0x43); Wire.endTransmission();
  Wire.requestFrom(address, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  if (address == 0x68) {
    RateRoll1 = (float)GyroX / 65.5;
    RatePitch1 = (float)GyroY / 65.5;
    RateYaw1 = (float)GyroZ / 65.5;
    AccX1 = (float)AccXLSB / 4096 - 0.05;
    AccY1 = (float)AccYLSB / 4096 + 0.01;
    AccZ1 = (float)AccZLSB / 4096 - 0.11;
    AngleRoll1 = atan2(AccY1, sqrt(AccX1 * AccX1 + AccZ1 * AccZ1)) * (180.0 / PI);
    AnglePitch1 = -atan2(AccX1, sqrt(AccY1 * AccY1 + AccZ1 * AccZ1)) * (180.0 / PI);
  } else if (address == 0x69) {
    RateRoll2 = (float)GyroX / 65.5;
    RatePitch2 = (float)GyroY / 65.5;
    RateYaw2 = (float)GyroZ / 65.5;
    AccX2 = (float)AccXLSB / 4096 - 0.05;
    AccY2 = (float)AccYLSB / 4096 + 0.01;
    AccZ2 = (float)AccZLSB / 4096 - 0.11;
    AngleRoll2 = atan2(AccY2, sqrt(AccX2 * AccX2 + AccZ2 * AccZ2)) * (180.0 / PI);
    AnglePitch2 = -atan2(AccX2, sqrt(AccY2 * AccY2 + AccZ2 * AccZ2)) * (180.0 / PI);
  }
}

// ======== SETUP ========
void setup() {
  Serial.begin(57600);
  ss.begin(GPSBaud);
  Wire.begin();
  Wire.setClock(400000);
  delay(250);

  // Wake up both MPU6050s
  Wire.beginTransmission(0x68); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();
  Wire.beginTransmission(0x69); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();

  // Calibrate gyros
  for (int i = 0; i < 2000; i++) {
    gyro_signals(0x68);
    RateCalibrationRoll1 += RateRoll1;
    RateCalibrationPitch1 += RatePitch1;
    RateCalibrationYaw1 += RateYaw1;

    gyro_signals(0x69);
    RateCalibrationRoll2 += RateRoll2;
    RateCalibrationPitch2 += RatePitch2;
    RateCalibrationYaw2 += RateYaw2;
    delay(1);
  }

  RateCalibrationRoll1 /= 2000; RateCalibrationPitch1 /= 2000; RateCalibrationYaw1 /= 2000;
  RateCalibrationRoll2 /= 2000; RateCalibrationPitch2 /= 2000; RateCalibrationYaw2 /= 2000;

  LoopTimer = micros();
}

// ======== LOOP ========
void loop() {
  // --- Read GPS Data ---
  while (ss.available() > 0) {
    gps.encode(ss.read());
  }

  // --- Read MPU6050 Data ---
  gyro_signals(0x68);
  gyro_signals(0x69);

  RateRoll1 -= RateCalibrationRoll1;
  RatePitch1 -= RateCalibrationPitch1;
  RateYaw1 -= RateCalibrationYaw1;

  RateRoll2 -= RateCalibrationRoll2;
  RatePitch2 -= RateCalibrationPitch2;
  RateYaw2 -= RateCalibrationYaw2;

  kalman_1d(KalmanAngleRoll1, KalmanUncertaintyAngleRoll1, RateRoll1, AngleRoll1);
  kalman_1d(KalmanAnglePitch1, KalmanUncertaintyAnglePitch1, RatePitch1, AnglePitch1);
  kalman_1d(KalmanAngleYaw1, KalmanUncertaintyAngleYaw1, RateYaw1, KalmanAngleYaw1);

  kalman_1d(KalmanAngleRoll2, KalmanUncertaintyAngleRoll2, RateRoll2, AngleRoll2);
  kalman_1d(KalmanAnglePitch2, KalmanUncertaintyAnglePitch2, RatePitch2, AnglePitch2);
  kalman_1d(KalmanAngleYaw2, KalmanUncertaintyAngleYaw2, RateYaw2, KalmanAngleYaw2);

  // --- Output to Serial Monitor ---
  Serial.println("========== SENSOR DATA ==========");

  // GPS Output
  if (gps.location.isUpdated()) {
    Serial.print("Latitude: "); Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: "); Serial.println(gps.location.lng(), 6);
    Serial.print("Date: ");
    Serial.print(gps.date.day()); Serial.print("/");
    Serial.print(gps.date.month()); Serial.print("/");
    Serial.println(gps.date.year());
    Serial.print("Time: ");
    Serial.print(gps.time.hour()); Serial.print(":");
    Serial.print(gps.time.minute()); Serial.print(":");
    Serial.println(gps.time.second());
    Serial.print("Speed (m/s): "); Serial.println(gps.speed.mps());
    Serial.print("Altitude (m): "); Serial.println(gps.altitude.meters());
  }

  // MPU6050 Output
  Serial.print("MPU1 - Roll: "); Serial.print(KalmanAngleRoll1);
  Serial.print(" | Pitch: "); Serial.print(KalmanAnglePitch1);
  Serial.print(" | Yaw: "); Serial.println(KalmanAngleYaw1);

  Serial.print("MPU2 - Roll: "); Serial.print(KalmanAngleRoll2);
  Serial.print(" | Pitch: "); Serial.print(KalmanAnglePitch2);
  Serial.print(" | Yaw: "); Serial.println(KalmanAngleYaw2);

  Serial.println();

  // Maintain loop timing
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}
