/*
  CUBESAT MPU6050 two sensor code (one address at 0x68 and one at 0x69)
  To make the MPU have different I2C adress, please connect AD0 pin <-> 5V in one of them to make one adress high (0x69)
  while another one connect AD0 <-> gnd to make it adress low (0x68)

  this code will show 6 results [row1,pitch1,yaw1,row2,pitch2,yaw2]

*/



#include <Wire.h>

// Gyroscope calibration offsets for both sensors
float RateCalibrationRoll1 = 0, RateCalibrationPitch1 = 0, RateCalibrationYaw1 = 0;
float RateCalibrationRoll2 = 0, RateCalibrationPitch2 = 0, RateCalibrationYaw2 = 0;

// Gyroscope rates for both sensors
float RateRoll1 = 0, RatePitch1 = 0, RateYaw1 = 0;
float RateRoll2 = 0, RatePitch2 = 0, RateYaw2 = 0;

// Accelerometer readings for both sensors
float AccX1 = 0, AccY1 = 0, AccZ1 = 0;
float AccX2 = 0, AccY2 = 0, AccZ2 = 0;

// Angle calculations for both sensors
float AngleRoll1 = 0, AnglePitch1 = 0;
float AngleRoll2 = 0, AnglePitch2 = 0;

// Kalman filter variables for both sensors
float KalmanAngleRoll1 = 0, KalmanUncertaintyAngleRoll1 = 4;
float KalmanAnglePitch1 = 0, KalmanUncertaintyAnglePitch1 = 4;
float KalmanAngleYaw1 = 0, KalmanUncertaintyAngleYaw1 = 4;

float KalmanAngleRoll2 = 0, KalmanUncertaintyAngleRoll2 = 4;
float KalmanAnglePitch2 = 0, KalmanUncertaintyAnglePitch2 = 4;
float KalmanAngleYaw2 = 0, KalmanUncertaintyAngleYaw2 = 4;

float Kalman1DOutput[] = {0, 0};

// Timing
uint32_t LoopTimer = 0;
float dt = 0.004; // 4ms loop time

void kalman_1d(float& KalmanState, float& KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState = KalmanState + dt * KalmanInput;
  KalmanUncertainty = KalmanUncertainty + dt * dt * 16; // Assuming process noise variance of 16
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 9); // Assuming measurement noise variance of 9
  KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
}

void gyro_signals(uint8_t address) {
  // Configure MPU6050 for a given I2C address
  Wire.beginTransmission(address);
  Wire.write(0x1A); // Sample rate divider
  Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(0x1C); // Accelerometer configuration
  Wire.write(0x10);
  Wire.endTransmission();

  // Read accelerometer data
  Wire.beginTransmission(address);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(address, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // Read gyroscope data
  Wire.beginTransmission(address);
  Wire.write(0x1B); // Gyroscope configuration
  Wire.write(0x08);
  Wire.endTransmission();

  Wire.beginTransmission(address);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(address, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  // Convert to physical units
  if (address == 0x68) {
    RateRoll1 = (float)GyroX / 65.5;
    RatePitch1 = (float)GyroY / 65.5;
    RateYaw1 = (float)GyroZ / 65.5;

    AccX1 = (float)AccXLSB / 4096 - 0.05;
    AccY1 = (float)AccYLSB / 4096 + 0.01;
    AccZ1 = (float)AccZLSB / 4096 - 0.11;

    // Calculate angles from accelerometer for sensor 1
    AngleRoll1 = atan2(AccY1, sqrt(AccX1 * AccX1 + AccZ1 * AccZ1)) * (180.0 / PI);
    AnglePitch1 = -atan2(AccX1, sqrt(AccY1 * AccY1 + AccZ1 * AccZ1)) * (180.0 / PI);
  }
  else if (address == 0x69) {
    RateRoll2 = (float)GyroX / 65.5;
    RatePitch2 = (float)GyroY / 65.5;
    RateYaw2 = (float)GyroZ / 65.5;

    AccX2 = (float)AccXLSB / 4096 - 0.05;
    AccY2 = (float)AccYLSB / 4096 + 0.01;
    AccZ2 = (float)AccZLSB / 4096 - 0.11;

    // Calculate angles from accelerometer for sensor 2
    AngleRoll2 = atan2(AccY2, sqrt(AccX2 * AccX2 + AccZ2 * AccZ2)) * (180.0 / PI);
    AnglePitch2 = -atan2(AccX2, sqrt(AccY2 * AccY2 + AccZ2 * AccZ2)) * (180.0 / PI);
  }
}

void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  // Wake up MPU6050 (both sensors)
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  Wire.beginTransmission(0x69);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Calibrate gyroscopes for both sensors
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
  RateCalibrationRoll1 /= 2000;
  RateCalibrationPitch1 /= 2000;
  RateCalibrationYaw1 /= 2000;
  RateCalibrationRoll2 /= 2000;
  RateCalibrationPitch2 /= 2000;
  RateCalibrationYaw2 /= 2000;

  LoopTimer = micros();
}

void loop() {
  gyro_signals(0x68); // Get data from MPU at address 0x68
  gyro_signals(0x69); // Get data from MPU at address 0x69

  // Remove calibration offsets for both sensors
  RateRoll1 -= RateCalibrationRoll1;
  RatePitch1 -= RateCalibrationPitch1;
  RateYaw1 -= RateCalibrationYaw1;

  RateRoll2 -= RateCalibrationRoll2;
  RatePitch2 -= RateCalibrationPitch2;
  RateYaw2 -= RateCalibrationYaw2;

  // Apply Kalman filter for both sensors
  kalman_1d(KalmanAngleRoll1, KalmanUncertaintyAngleRoll1, RateRoll1, AngleRoll1);
  kalman_1d(KalmanAnglePitch1, KalmanUncertaintyAnglePitch1, RatePitch1, AnglePitch1);
  kalman_1d(KalmanAngleYaw1, KalmanUncertaintyAngleYaw1, RateYaw1, KalmanAngleYaw1); // No direct measurement for yaw

  kalman_1d(KalmanAngleRoll2, KalmanUncertaintyAngleRoll2, RateRoll2, AngleRoll2);
  kalman_1d(KalmanAnglePitch2, KalmanUncertaintyAnglePitch2, RatePitch2, AnglePitch2);
  kalman_1d(KalmanAngleYaw2, KalmanUncertaintyAngleYaw2, RateYaw2, KalmanAngleYaw2); // No direct measurement for yaw

  // Output angles for both sensors
  Serial.print(KalmanAngleRoll1);
  Serial.print(" | ");
  Serial.print(KalmanAnglePitch1);
  Serial.print(" | ");
  Serial.print(KalmanAngleYaw1);
  Serial.print(" | ");
  Serial.print(KalmanAngleRoll2);
  Serial.print(" | ");
  Serial.print(KalmanAnglePitch2);
  Serial.print(" | ");
  Serial.println(KalmanAngleYaw2);

  // Wait for next loop
  while (micros() - LoopTimer < 4000);
  LoopTimer = micros();
}
