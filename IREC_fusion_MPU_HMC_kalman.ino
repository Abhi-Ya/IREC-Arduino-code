/*
IREC 2025 MPU + HMC fusion with Kalman filter:
improve the limitation of 
 - accumurating of error
 - interference of magnetic wave
*/

#include <Wire.h>
 
// --- DEVICE ADDRESSES ---
#define MPU6050_ADDR 0x68   // MPU6050 I2C address
#define HMC5883L_ADDR 0x1E  // HMC5883L I2C address
 
// --- MPU6050 REGISTERS ---
#define MPU6050_REG_ACCEL_XOUT_H 0x3B // Starting register for accelerometer readings
#define MPU6050_REG_PWR_MGMT_1   0x6B // Power management register
#define MPU6050_REG_GYRO_XOUT_H  0x43 // Starting register for gyroscope readings
 
// --- SENSITIVITY SCALES ---
#define ACCEL_SCALE 16384.0 // Accelerometer scale for ±2g (16-bit)
#define GYRO_SCALE 131.0    // Gyroscope scale for ±250°/s (16-bit)
#define MAG_SCALE 0.92      // Magnetometer scale for ±1.3 Gauss
 
// --- CALIBRATION OFFSETS ---
double accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0; // Accelerometer offsets
double gyroXOffset = 0, gyroYOffset = 0, gyroZOffset = 0;    // Gyroscope offsets
double magMinX = 0, magMaxX = 0, magMinY = 0, magMaxY = 0, magMinZ = 0, magMaxZ = 0; // Magnetometer calibration
 
// --- FILTER VARIABLES ---
double pitch = 0, roll = 0, yaw = 0;  // Orientation angles (degrees)
double dt = 0.02;                     // Loop time in seconds (50 Hz)
 
// --- KALMAN FILTER PARAMETERS ---
double Q_angle = 0.001;  // Process noise variance for the accelerometer
double Q_bias = 0.003;   // Process noise variance for the gyroscope bias
double R_measure = 0.03; // Measurement noise variance
double angle = 0, bias = 0, rate = 0; // Kalman filter state variables
double P[2][2] = {{0, 0}, {0, 0}};    // Error covariance matrix
 
unsigned long lastTime; // For calculating loop time
 
void setup() {
  // --- INITIAL SETUP ---
  Serial.begin(115200); // Initialize serial communication for debugging
  Wire.begin();         // Initialize I2C communication
 
  // --- SENSOR INITIALIZATION ---
  MPU6050_init();   // Initialize MPU6050
  HMC5883L_init();  // Initialize HMC5883L
 
  // --- SENSOR CALIBRATION ---
  calibrate_MPU6050();  // Calibrate accelerometer and gyroscope
  calibrate_HMC5883L(); // Calibrate magnetometer
 
  lastTime = millis(); // Set the initial time for the loop
}
 
void loop() {
  double ax, ay, az, gx, gy, gz; // Variables to store accelerometer and gyroscope data
  double mx, my, mz;            // Variables to store magnetometer data
 
  // --- READ SENSOR DATA ---
  read_MPU6050(ax, ay, az, gx, gy, gz); // Get accelerometer and gyroscope data
  read_HMC5883L(mx, my, mz);           // Get magnetometer data
 
  // --- CALCULATE TIME STEP ---
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0; // Calculate time in seconds
  if (dt == 0) dt = 0.001;               // Prevent division by zero
  lastTime = currentTime;
 
  // --- KALMAN FILTER FOR PITCH AND ROLL ---
  pitch = Kalman_filter(pitch, gx, atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / PI);
  roll = Kalman_filter(roll, gy, atan2(ay, az) * 180.0 / PI);
  
  // --- YAW CALCULATION USING MAGNETOMETER ---
  yaw = atan2(my, mx); 
  float declinationAngle = -0.01221; // always calibrate your declination on certain position using magnetic delication website !!!!!!!!!!!!!!!!!!!
  yaw += declinationAngle;        
 
  // Normalize yaw to 0-360 degrees
  if (yaw < 0) yaw += 2 * PI;
  if (yaw > 2 * PI) yaw -= 2 * PI;
  yaw = yaw * 180.0 / PI; // Convert yaw to degrees
 
  // --- PRINT RESULTS ---
  Serial.print(pitch); Serial.print("\t");
  Serial.print(roll); Serial.print("\t");
  Serial.println(yaw); Serial.print("\t");
 
  delay(20); // Maintain 50 Hz loop frequency
}
 
// --- SENSOR INITIALIZATION FUNCTIONS ---
void MPU6050_init() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_PWR_MGMT_1); // Select power management register
  Wire.write(0);                      // Wake up MPU6050
  Wire.endTransmission(true);
}
 
void HMC5883L_init() {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x00); // Configuration Register A
  Wire.write(0x70); // 8-average, 15 Hz default, normal measurement
  Wire.endTransmission();
 
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x01); // Configuration Register B
  Wire.write(0x20); // Gain = 5 (±1.3 Gauss)
  Wire.endTransmission();
 
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x02); // Mode Register
  Wire.write(0x00); // Continuous measurement mode
  Wire.endTransmission();
}
 
// --- SENSOR CALIBRATION FUNCTIONS ---
void calibrate_MPU6050() {
  Serial.println("Calibrating MPU6050...");
  double ax, ay, az, gx, gy, gz;
  int numSamples = 100;
 
  for (int i = 0; i < numSamples; i++) {
    read_MPU6050(ax, ay, az, gx, gy, gz);
    accelOffsetX += ax;
    accelOffsetY += ay;
    accelOffsetZ += az;
    gyroXOffset += gx;
    gyroYOffset += gy;
    gyroZOffset += gz;
    delay(10);
  }
 
  accelOffsetX /= numSamples;
  accelOffsetY /= numSamples;
  accelOffsetZ /= numSamples;
  gyroXOffset /= numSamples;
  gyroYOffset /= numSamples;
  gyroZOffset /= numSamples;
 
  // Adjust for gravity on the Z-axis
  accelOffsetZ -= 1.0;
 
  Serial.println("MPU6050 Calibration Complete.");
}
 
void calibrate_HMC5883L() {
  Serial.println("Calibrating HMC5883L... Rotate the sensor in all directions.");
  double mx, my, mz;
 
  magMinX = magMinY = magMinZ = 1e6;
  magMaxX = magMaxY = magMaxZ = -1e6;
 
  unsigned long startTime = millis();
  while (millis() - startTime < 10000) { // 10 seconds for calibration
    read_HMC5883L(mx, my, mz);
 
    if (mx < magMinX) magMinX = mx;
    if (mx > magMaxX) magMaxX = mx;
    if (my < magMinY) magMinY = my;
    if (my > magMaxY) magMaxY = my;
    if (mz < magMinZ) magMinZ = mz;
    if (mz > magMaxZ) magMaxZ = mz;
 
    Serial.print("."); // Print a dot every 100 ms to indicate progress
    delay(100);
  }
  Serial.println("\nHMC5883L Calibration Complete.");
}
 
// --- SENSOR READING FUNCTIONS ---
void read_MPU6050(double &ax, double &ay, double &az, double &gx, double &gy, double &gz) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_REG_ACCEL_XOUT_H); // Starting register for Accel Readings
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 14, true);
 
  ax = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE - accelOffsetX;
  ay = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE - accelOffsetY;
  az = (Wire.read() << 8 | Wire.read()) / ACCEL_SCALE - accelOffsetZ;
  gx = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE - gyroXOffset;
  gy = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE - gyroYOffset;
  gz = (Wire.read() << 8 | Wire.read()) / GYRO_SCALE - gyroZOffset;
}
 
void read_HMC5883L(double &mx, double &my, double &mz) {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x03); // Starting register for magnetometer readings
  Wire.endTransmission(false);
  Wire.requestFrom(HMC5883L_ADDR, 6, true);
 
  int16_t x = Wire.read() << 8 | Wire.read();
  int16_t z = Wire.read() << 8 | Wire.read();
  int16_t y = Wire.read() << 8 | Wire.read();
 
  // Apply calibration offsets and scaling
  mx = (x - (magMinX + magMaxX) / 2) * MAG_SCALE;
  my = (y - (magMinY + magMaxY) / 2) * MAG_SCALE;
  mz = (z - (magMinZ + magMaxZ) / 2) * MAG_SCALE;
}
 
// --- KALMAN FILTER FUNCTION ---
double Kalman_filter(double angle, double gyroRate, double accelAngle) {
  // Predict
  rate = gyroRate - bias;
  angle += dt * rate;
 
  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;
 
  // Update
  double S = P[0][0] + R_measure; // Estimate error
  double K[2];                    // Kalman gain
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;
 
  double y = accelAngle - angle; // Angle difference
  angle += K[0] * y;
  bias += K[1] * y;
 
  double P00_temp = P[0][0];
  double P01_temp = P[0][1];
 
  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
 
  return angle;
}
