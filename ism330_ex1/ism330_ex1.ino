/*
  example1-basic, mods for Adafruit Feather ESP32-S2 TFT

  This example shows the basic settings and functions for retrieving accelerometer
	and gyroscopic data.
	Please refer to the header file for more possible settings, found here:
	..\SparkFun_6DoF_ISM330DHCX_Arduino_Library\src\sfe_ism330dhcx_defs.h

  Written by Elias Santistevan @ SparkFun Electronics, August 2022

	Product:

		https://www.sparkfun.com/products/19764

  Repository:

		https://github.com/sparkfun/SparkFun_6DoF_ISM330DHCX_Arduino_Library

  SparkFun code, firmware, and software is released under the MIT
	License	(http://opensource.org/licenses/MIT).
*/

// tested on Adafruit Feather ESP32_S2 TFT

#include <Wire.h>
#include "SparkFun_ISM330DHCX.h"

SparkFun_ISM330DHCX ISM330;

// Structs for X,Y,Z data
sfe_ism_data_t acc;
sfe_ism_data_t gyr;

void setup() {
  
// power up I2C port
#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
#endif

  Wire.begin();

  Serial.begin(115200);
  while (!Serial);

  if ( !ISM330.begin(0x6A) ) {  //0x6B is default with SFE library
    Serial.println("ISM330 not detected.");
    while (1) delay(1);
  }

  Serial.println("Reset.");
  // Reset the device to default settings. This if helpful is you're doing multiple
  // uploads testing different settings.
  ISM330.deviceReset();

  // Wait for it to finish reseting
  while ( !ISM330.getDeviceReset() ) {
    delay(1);
  }


  Serial.println("Applying settings.");

  ISM330.setDeviceConfig();
  ISM330.setBlockDataUpdate();

  // Set the output data rate and precision of the accelerometer
  ISM330.setAccelDataRate(ISM_XL_ODR_104Hz);
  ISM330.setAccelFullScale(ISM_2g);

  // Set the output data rate and precision of the gyroscope
  ISM330.setGyroDataRate(ISM_GY_ODR_104Hz);
  ISM330.setGyroFullScale(ISM_500dps);
  /*
  	// Turn on the accelerometer's filter and apply settings.
  	ISM330.setAccelFilterLP2();
  	ISM330.setAccelSlopeFilter(ISM_LP_ODR_DIV_100);

  	// Turn on the gyroscope's filter and apply settings.
  	ISM330.setGyroFilterLP1();
  	ISM330.setGyroLP1Bandwidth(ISM_MEDIUM);
  */

}

// using SFE library,
// getAccel() returns milli_g, regardless of scale setting
// getGyro() returns milli_DPS, " 

void loop() {

  // Check if both gyroscope and accelerometer data are available.
  if ( ISM330.checkStatus() ) {
    ISM330.getAccel(&acc);
    ISM330.getGyro(&gyr);
    Serial.print("Axyz: ");
    Serial.print(acc.xData);
    Serial.print(" ");
    Serial.print(acc.yData);
    Serial.print(" ");
    Serial.print(acc.zData);
    Serial.println();
    Serial.print("Gxyz: ");
    Serial.print(gyr.xData);
    Serial.print(" ");
    Serial.print(gyr.yData);
    Serial.print(" ");
    Serial.print(gyr.zData);
    Serial.println();
  }

  delay(100);
}
