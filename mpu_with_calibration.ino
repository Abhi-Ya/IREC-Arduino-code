#include <Wire.h>

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// Raw sensor values
int16_t rawAccX, rawAccY, rawAccZ;
int16_t rawGyroX, rawGyroY, rawGyroZ;

// Calibration offsets
float accOffsetX = 0, accOffsetY = 0, accOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

// Scaled sensor values
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

// Orientation angles
float pitch, roll;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Fast I2C

  initializeMPU6050();
  calibrateSensors();

  Serial.println("MPU6050 Ready.");
}

void loop() {
  readMPU6050();

  // Convert raw to g's and deg/s, apply calibration
  accX = (rawAccX / 16384.0) - accOffsetX;
  accY = (rawAccY / 16384.0) - accOffsetY;
  accZ = (rawAccZ / 16384.0) - accOffsetZ;

  gyroX = (rawGyroX / 131.0) - gyroOffsetX;
  gyroY = (rawGyroY / 131.0) - gyroOffsetY;
  gyroZ = (rawGyroZ / 131.0) - gyroOffsetZ;

  // Compute pitch and roll
  pitch = atan2(-accX, sqrt(accY * accY + accZ * accZ)) * 180.0 / PI;
  roll = atan2(accY, accZ) * 180.0 / PI;

  // Print angles [Pitch: ROll]

  //Serial.print("Pitch: ");
  Serial.print(pitch); Serial.print("\t");
  //Serial.print(" °\tRoll: ");
  Serial.println(roll); Serial.print("\t");
  //Serial.println(" °");

  delay(200);
}

void initializeMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B); // Power management register
  Wire.write(0);    // Wake up MPU6050
  Wire.endTransmission(true);
}

void readMPU6050() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // Starting register for Accel data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true); // Read 14 bytes

  rawAccX = Wire.read() << 8 | Wire.read();
  rawAccY = Wire.read() << 8 | Wire.read();
  rawAccZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // Skip Temp
  rawGyroX = Wire.read() << 8 | Wire.read();
  rawGyroY = Wire.read() << 8 | Wire.read();
  rawGyroZ = Wire.read() << 8 | Wire.read();
}

void calibrateSensors() {
  Serial.println("Calibrating... Keep the sensor still");
  const int samples = 1000;
  long accXSum = 0, accYSum = 0, accZSum = 0;
  long gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;

  for (int i = 0; i < samples; i++) {
    readMPU6050();
    accXSum += rawAccX;
    accYSum += rawAccY;
    accZSum += rawAccZ;
    gyroXSum += rawGyroX;
    gyroYSum += rawGyroY;
    gyroZSum += rawGyroZ;
    delay(3);
  }

  accOffsetX = (float)(accXSum / samples) / 16384.0;
  accOffsetY = (float)(accYSum / samples) / 16384.0;
  accOffsetZ = ((float)(accZSum / samples) / 16384.0) - 1.0; // Adjust for gravity

  gyroOffsetX = (float)(gyroXSum / samples) / 131.0;
  gyroOffsetY = (float)(gyroYSum / samples) / 131.0;
  gyroOffsetZ = (float)(gyroZSum / samples) / 131.0;

  Serial.println("Calibration complete.");
}
