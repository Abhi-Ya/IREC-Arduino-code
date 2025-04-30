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

// === GPS Setup ===
static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// === Multiplexer ===
#define TCAADDR 0x70
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// === BME280 on Channel 1 ===
Adafruit_BME280 bme;
bool bme_ok = false;

// === MPU6050 Shared Code ===
struct MPU6050Data {
  uint8_t channel;
  uint8_t address;
  bool ok;
  float RateRoll, RatePitch, RateYaw;
  float AccX, AccY, AccZ;
  float AngleRoll, AnglePitch;
  float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 4.0;
  float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 4.0;
  float RateCalibrationRoll = 0, RateCalibrationPitch = 0, RateCalibrationYaw = 0;
  bool read_ok = false;
};

MPU6050Data mpu1 = {7, 0x68, true};
MPU6050Data mpu2 = {6, 0x69, true};

float Kalman1DOutput[2];

void kalman_1d(float& KalmanState, float& KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  KalmanState += 0.004 * KalmanInput;
  KalmanUncertainty += 0.004 * 0.004 * 16;
  float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 9);
  KalmanState += KalmanGain * (KalmanMeasurement - KalmanState);
  KalmanUncertainty *= (1 - KalmanGain);
}

bool readMPU6050(MPU6050Data &mpu) {
  tcaSelect(mpu.channel);
  
  Wire.beginTransmission(mpu.address);
  Wire.write(0x3B);
  if (Wire.endTransmission() != 0) return false;
  Wire.requestFrom(mpu.address, 6);
  if (Wire.available() < 6) return false;
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(mpu.address);
  Wire.write(0x43);
  if (Wire.endTransmission() != 0) return false;
  Wire.requestFrom(mpu.address, 6);
  if (Wire.available() < 6) return false;
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  mpu.RateRoll = (float)GyroX / 65.5 - mpu.RateCalibrationRoll;
  mpu.RatePitch = (float)GyroY / 65.5 - mpu.RateCalibrationPitch;
  mpu.RateYaw = (float)GyroZ / 65.5 - mpu.RateCalibrationYaw;

  mpu.AccX = (float)AccXLSB / 4096 - 0.05;
  mpu.AccY = (float)AccYLSB / 4096 + 0.01;
  mpu.AccZ = (float)AccZLSB / 4096 - 0.11;

  mpu.AngleRoll = atan(mpu.AccY / sqrt(mpu.AccX * mpu.AccX + mpu.AccZ * mpu.AccZ)) * 180 / PI;
  mpu.AnglePitch = -atan(mpu.AccX / sqrt(mpu.AccY * mpu.AccY + mpu.AccZ * mpu.AccZ)) * 180 / PI;

  kalman_1d(mpu.KalmanAngleRoll, mpu.KalmanUncertaintyAngleRoll, mpu.RateRoll, mpu.AngleRoll);
  kalman_1d(mpu.KalmanAnglePitch, mpu.KalmanUncertaintyAnglePitch, mpu.RatePitch, mpu.AnglePitch);

  return true;
}

void calibrateMPU(MPU6050Data& mpu) {
  tcaSelect(mpu.channel);
  for (int i = 0; i < 1000; i++) {
    if (!readMPU6050(mpu)) {
      mpu.ok = false;
      return;
    }
    mpu.RateCalibrationRoll += mpu.RateRoll;
    mpu.RateCalibrationPitch += mpu.RatePitch;
    mpu.RateCalibrationYaw += mpu.RateYaw;
    delay(1);
  }
  mpu.RateCalibrationRoll /= 1000;
  mpu.RateCalibrationPitch /= 1000;
  mpu.RateCalibrationYaw /= 1000;
  mpu.ok = true;
}

void setup() {
  Serial.begin(57600);
  Wire.begin();
  Wire.setClock(400000);
  ss.begin(GPSBaud);

  // Setup BME280
  tcaSelect(1);
  bme_ok = bme.begin(0x76);

  // Wake up both MPUs
  tcaSelect(mpu1.channel);
  Wire.beginTransmission(mpu1.address);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  tcaSelect(mpu2.channel);
  Wire.beginTransmission(mpu2.address);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission();

  delay(250);

  calibrateMPU(mpu1);
  calibrateMPU(mpu2);
}

uint32_t LoopTimer;

void loop() {
  // Read GPS
  while (ss.available() > 0) gps.encode(ss.read());

  // Read Sensors
  mpu1.read_ok = mpu1.ok && readMPU6050(mpu1);
  mpu2.read_ok = mpu2.ok && readMPU6050(mpu2);

  tcaSelect(1); // Switch to BME280
  float humidity = bme_ok ? bme.readHumidity() : -1;
  float temperature = bme_ok ? bme.readTemperature() : -1;
  float pressure = bme_ok ? bme.readPressure() / 100.0F : -1;
  float altitude = bme_ok ? bme.readAltitude(1013.25) : -1;

  // === Output ===
  Serial.print("BME | H: "); Serial.print(humidity != -1 ? String(humidity) : "--");
  Serial.print("% | T: "); Serial.print(temperature != -1 ? String(temperature) : "--");
  Serial.print("C | P: "); Serial.print(pressure != -1 ? String(pressure) : "--");
  Serial.print("hPa | Alt: "); Serial.print(altitude != -1 ? String(altitude) : "--"); Serial.println("m");

  Serial.print("MPU1 | Roll: "); Serial.print(mpu1.read_ok ? mpu1.KalmanAngleRoll : --); Serial.print("° ");
  Serial.print("| Pitch: "); Serial.println(mpu1.read_ok ? mpu1.KalmanAnglePitch : --);

  Serial.print("MPU2 | Roll: "); Serial.print(mpu2.read_ok ? mpu2.KalmanAngleRoll : --); Serial.print("° ");
  Serial.print("| Pitch: "); Serial.println(mpu2.read_ok ? mpu2.KalmanAnglePitch : --);

  if (gps.location.isUpdated()) {
    Serial.print("GPS | Lat: "); Serial.print(gps.location.lat(), 6);
    Serial.print(" | Lon: "); Serial.print(gps.location.lng(), 6);
    Serial.print(" | Speed: "); Serial.print(gps.speed.mps());
    Serial.print(" m/s | Alt: "); Serial.println(gps.altitude.meters());
  } else {
    Serial.println("GPS: --");
  }

  delay(10);
}
