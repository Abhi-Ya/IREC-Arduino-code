/*
  IREC code which assembled MULTIPLEXER with connecting to ARDUINO UNO and 1 BME280, 2 MPU6050, and 1 GPS sensor.
  
  !! NOTED that this code not include reporting this data to the microSD card yet. just used to test that the circuit and connection works fine please ;)

  the circuit connection is like what I drew. 
  BME280 on channel 1

  MPU6050 at address 0x68 on channel 7

  MPU6050 at address 0x69 on channel 6

  GPS on digital pins 3 (RX) and 4 (TX)
  
*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// === TCA9548A Multiplexer Helper ===
void tca_select(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(0x70);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// === BME280 Setup ===
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;
bool bme_ok = false;

// === MPU6050 #1 (0x68 on channel 6) Variables ===
float RateRoll1, RatePitch1, RateYaw1;
float RateCalibrationRoll1, RateCalibrationPitch1, RateCalibrationYaw1;
float AccX1, AccY1, AccZ1;
float AngleRoll1, AnglePitch1;
float KalmanAngleRoll1 = 0, KalmanUncertaintyAngleRoll1 = 4;
float KalmanAnglePitch1 = 0, KalmanUncertaintyAnglePitch1 = 4;
bool mpu1_ok = true;

// === MPU6050 #2 (0x69 on channel 7) Variables ===
float RateRoll2, RatePitch2, RateYaw2;
float RateCalibrationRoll2, RateCalibrationPitch2, RateCalibrationYaw2;
float AccX2, AccY2, AccZ2;
float AngleRoll2, AnglePitch2;
float KalmanAngleRoll2 = 0, KalmanUncertaintyAngleRoll2 = 4;
float KalmanAnglePitch2 = 0, KalmanUncertaintyAnglePitch2 = 4;
bool mpu2_ok = true;

// === Kalman Filter Output Buffer ===
float Kalman1DOutput[] = {0, 0};

// === GPS Setup ===
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

uint32_t LoopTimer;

// === Kalman Filter Function ===
void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState += 0.004 * KalmanInput;
  KalmanUncertainty += 0.004 * 0.004 * 16;
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 9);
  KalmanState += KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty *= (1 - KalmanGain);
  Kalman1DOutput[0] = KalmanState;
  Kalman1DOutput[1] = KalmanUncertainty;
}

// === Read MPU6050 Data (Generic Function) ===
bool read_mpu6050(uint8_t channel, uint8_t addr,
                  float &RateRoll, float &RatePitch, float &RateYaw,
                  float &AccX, float &AccY, float &AccZ,
                  float &AngleRoll, float &AnglePitch) {
  tca_select(channel);
  Wire.beginTransmission(addr);
  Wire.write(0x3B);
  if (Wire.endTransmission() != 0) return false;
  Wire.requestFrom(addr, 6);
  if (Wire.available() < 6) return false;
  int16_t ax = Wire.read() << 8 | Wire.read();
  int16_t ay = Wire.read() << 8 | Wire.read();
  int16_t az = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(addr);
  Wire.write(0x43);
  if (Wire.endTransmission() != 0) return false;
  Wire.requestFrom(addr, 6);
  if (Wire.available() < 6) return false;
  int16_t gx = Wire.read() << 8 | Wire.read();
  int16_t gy = Wire.read() << 8 | Wire.read();
  int16_t gz = Wire.read() << 8 | Wire.read();

  RateRoll = (float)gx / 65.5;
  RatePitch = (float)gy / 65.5;
  RateYaw = (float)gz / 65.5;

  AccX = (float)ax / 4096 - 0.05;
  AccY = (float)ay / 4096 + 0.01;
  AccZ = (float)az / 4096 - 0.11;

  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 180 / PI;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 180 / PI;

  return true;
}

void setup() {
  Serial.begin(57600);
  Wire.setClock(400000);
  Wire.begin();

  // === BME280 ===
  tca_select(1);
  bme_ok = bme.begin(0x76);

  // === MPU6050 #1 init (0x68 on channel 6) ===
  tca_select(6);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(250);

  // === MPU6050 #2 init (0x69 on channel 7) ===
  tca_select(7);
  Wire.beginTransmission(0x69);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(250);

  // === Calibration ===
  for (int i = 0; i < 2000; i++) {
    if (!read_mpu6050(6, 0x68, RateRoll1, RatePitch1, RateYaw1, AccX1, AccY1, AccZ1, AngleRoll1, AnglePitch1)) {
      mpu1_ok = false;
      break;
    }
    RateCalibrationRoll1 += RateRoll1;
    RateCalibrationPitch1 += RatePitch1;
    RateCalibrationYaw1 += RateYaw1;

    if (!read_mpu6050(7, 0x69, RateRoll2, RatePitch2, RateYaw2, AccX2, AccY2, AccZ2, AngleRoll2, AnglePitch2)) {
      mpu2_ok = false;
      break;
    }
    RateCalibrationRoll2 += RateRoll2;
    RateCalibrationPitch2 += RatePitch2;
    RateCalibrationYaw2 += RateYaw2;

    delay(1);
  }

  if (mpu1_ok) {
    RateCalibrationRoll1 /= 2000;
    RateCalibrationPitch1 /= 2000;
    RateCalibrationYaw1 /= 2000;
  }
  if (mpu2_ok) {
    RateCalibrationRoll2 /= 2000;
    RateCalibrationPitch2 /= 2000;
    RateCalibrationYaw2 /= 2000;
  }

  ss.begin(GPSBaud);
  LoopTimer = micros();
}

void loop() {
  while (ss.available() > 0) gps.encode(ss.read());

  // === Read and Process MPU1 ===
  bool mpu1_read_ok = false;
  if (mpu1_ok) {
    mpu1_read_ok = read_mpu6050(6, 0x68, RateRoll1, RatePitch1, RateYaw1, AccX1, AccY1, AccZ1, AngleRoll1, AnglePitch1);
    if (mpu1_read_ok) {
      RateRoll1 -= RateCalibrationRoll1;
      RatePitch1 -= RateCalibrationPitch1;
      kalman_1d(KalmanAngleRoll1, KalmanUncertaintyAngleRoll1, RateRoll1, AngleRoll1);
      KalmanAngleRoll1 = Kalman1DOutput[0];
      KalmanUncertaintyAngleRoll1 = Kalman1DOutput[1];

      kalman_1d(KalmanAnglePitch1, KalmanUncertaintyAnglePitch1, RatePitch1, AnglePitch1);
      KalmanAnglePitch1 = Kalman1DOutput[0];
      KalmanUncertaintyAnglePitch1 = Kalman1DOutput[1];
    }
  }

  // === Read and Process MPU2 ===
  bool mpu2_read_ok = false;
  if (mpu2_ok) {
    mpu2_read_ok = read_mpu6050(7, 0x69, RateRoll2, RatePitch2, RateYaw2, AccX2, AccY2, AccZ2, AngleRoll2, AnglePitch2);
    if (mpu2_read_ok) {
      RateRoll2 -= RateCalibrationRoll2;
      RatePitch2 -= RateCalibrationPitch2;
      kalman_1d(KalmanAngleRoll2, KalmanUncertaintyAngleRoll2, RateRoll2, AngleRoll2);
      KalmanAngleRoll2 = Kalman1DOutput[0];
      KalmanUncertaintyAngleRoll2 = Kalman1DOutput[1];

      kalman_1d(KalmanAnglePitch2, KalmanUncertaintyAnglePitch2, RatePitch2, AnglePitch2);
      KalmanAnglePitch2 = Kalman1DOutput[0];
      KalmanUncertaintyAnglePitch2 = Kalman1DOutput[1];
    }
  }

  // === Output ===
  tca_select(1);
  Serial.print(bme_ok ? String(bme.readHumidity()) : "--"); Serial.print("% | ");
  Serial.print(bme_ok ? String(bme.readTemperature()) : "--"); Serial.print("*C | ");
  Serial.print(bme_ok ? String(bme.readPressure() / 100.0F) : "--"); Serial.print("hPa | ");
  Serial.print(bme_ok ? String(bme.readAltitude(SEALEVELPRESSURE_HPA)) : "--"); Serial.print("m | ");

  Serial.print("MPU1: ");
  Serial.print(mpu1_read_ok ? String(KalmanAngleRoll1) : "--"); Serial.print("° / ");
  Serial.print(mpu1_read_ok ? String(KalmanAnglePitch1) : "--"); Serial.print("° | ");

  Serial.print("MPU2: ");
  Serial.print(mpu2_read_ok ? String(KalmanAngleRoll2) : "--"); Serial.print("° / ");
  Serial.print(mpu2_read_ok ? String(KalmanAnglePitch2) : "--"); Serial.print("° | ");

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

