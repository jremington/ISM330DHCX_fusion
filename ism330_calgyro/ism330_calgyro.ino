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

// number of data samples to average
#define GYRO_SAMPLES 500

SparkFun_ISM330DHCX ISM330;

// Structs for X,Y,Z data
sfe_ism_data_t acc;
sfe_ism_data_t gyr;

int loopcount = 0;
float gxyz_offsets[3] = {0};

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
  /*
    Serial.println("Reset.");
    // Reset the device to default settings. This if helpful is you're doing multiple
    // uploads testing different settings.
    ISM330.deviceReset();

    // Wait for it to finish reseting
    while ( !ISM330.getDeviceReset() ) {
      delay(1);
    }
  */
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
*/
  	// Turn on the gyroscope's filter and apply settings.
  	ISM330.setGyroFilterLP1();
  	ISM330.setGyroLP1Bandwidth(ISM_MEDIUM);
  
  Serial.println("Keep gyro still for offset calculation");
  delay(2000);
}

// using SFE library,
// getAccel() returns milli_g, regardless of scale setting
// getGyro() returns milli_DPS, "


void loop() {
  
  if (loopcount < GYRO_SAMPLES) {  //accumulate sums
    // Check if both gyroscope and accelerometer data are available.
    if ( ISM330.checkStatus() ) {
      ISM330.getGyro(&gyr);
      gxyz_offsets[0] += gyr.xData;
      gxyz_offsets[1] += gyr.yData;
      gxyz_offsets[2] += gyr.zData;
      loopcount++;
    }
  }

  if (loopcount == GYRO_SAMPLES) { //calculate and subtract offsets
    for (int i = 0; i < 3; i++) gxyz_offsets[i] /= (float)loopcount;
    Serial.print("Done, gyro offsets ");
    for (int i = 0; i < 3; i++) {
      Serial.print(gxyz_offsets[i], 1);
      Serial.print(", ");
    }
    Serial.println();
    Serial.println("Subtract and check:");
    loopcount++;  //one final increment
  }
  
  if (loopcount == GYRO_SAMPLES + 1) {
    if ( ISM330.checkStatus() ) {  //print corrected data for drift check
      ISM330.getGyro(&gyr);
      Serial.print("Gxyz: ");
      Serial.print(gyr.xData - gxyz_offsets[0],0);
      Serial.print(" ");
      Serial.print(gyr.yData - gxyz_offsets[1], 0);
      Serial.print(" ");
      Serial.println(gyr.zData - gxyz_offsets[2], 0);
      delay(300);
    }
  }
}
