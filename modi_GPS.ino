/*
   Rui Santos
   Complete Project Details: https://randomnerdtutorials.com
   Based on TinyGPS++ by Mikal Hart (https://github.com/mikalhart/TinyGPSPlus)
*/

#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 4, TXPin = 3;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  Serial.begin(57600); // default baud for gps is 9600 but i changed to 57600 to make it like the mpu na ka.
  ss.begin(GPSBaud);
}

void loop() {
  while (ss.available() > 0) {
    gps.encode(ss.read());

    if (gps.location.isUpdated()) {
      Serial.print("Latitude: ");
      Serial.println(gps.location.lat(), 6);

      Serial.print("Longitude: ");
      Serial.println(gps.location.lng(), 6);

      Serial.print("Date: ");
      Serial.print(gps.date.day());
      Serial.print("/");
      Serial.print(gps.date.month());
      Serial.print("/");
      Serial.println(gps.date.year());

      Serial.print("Time: ");
      Serial.print(gps.time.hour());
      Serial.print(":");
      Serial.print(gps.time.minute());
      Serial.print(":");
      Serial.println(gps.time.second());

      Serial.print("Speed (m/s): ");
      Serial.println(gps.speed.mps());

      Serial.print("Altitude (m): ");
      Serial.println(gps.altitude.meters());

      Serial.println();  // Extra line for readability
    }
  }
}
