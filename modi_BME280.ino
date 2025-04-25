#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;

void setup() {
	Serial.begin(57600); // default at 9600 but IREC will use 57600 baud

	if (!bme.begin(0x76)) { // default setting i2c of the BME280 is 0x76 (support most 2 adress: 76,77)
		Serial.println("Could not find a valid BME280 sensor, check wiring!");
		while (1);
	}
}

void loop() {

  // output [temperture,pressure,approx altitude, humidity]
  /*
  temperature in celcius unit
  pressure in hPa unit
  altitude calculated in metres unit
  humidity calculate in % humid unit
  
  
  */
	
	Serial.print(bme.readTemperature());
	Serial.print("*C | ");

	Serial.print(bme.readPressure() / 100.0F);
	Serial.print("hPa | ");

	Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
	Serial.print("m | ");

	Serial.print(bme.readHumidity());
	Serial.println("%");

	delay(1000);
}