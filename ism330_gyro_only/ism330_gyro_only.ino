/*

  "gyro only" Mahony integration for ISM330DHCX 6DOF sensor, accelerometer ignored.
  Angles printed are relative to start angles.

  S. J. Remington 4/2023

  tested with Adafruit Feather ESP32_S2 TFT
  and Arduino Pro Mini, 3.3V 8 MHz. Works great!

  Device library:
		https://github.com/sparkfun/SparkFun_6DoF_ISM330DHCX_Arduino_Library

*/

#include <Wire.h>
#include "SparkFun_ISM330DHCX.h"

SparkFun_ISM330DHCX ISM330;

// Structs for X,Y,Z data
sfe_ism_data_t acc;
sfe_ism_data_t gyr;


// if gyro is to calibrated upon startup, define the following:
#define CAL_GYRO

float gxyz_offsets[3] = {0};
// otherwise, define the gyro offsets explicitly
//float gxyz_offsets[3] = {131.0, -579.0, -183.0};

// library returns milliDPS
float RadiansPerSecond = PI / 180000.0; //conversion factor for Mahony filter
float Gxyz[3] = {0}; //arrays for Mahony filter

float q[4] = {1.0, 0.0, 0.0, 0.0}; //initialize quaternion, global access. X points at yaw=0

// example code just converts and prints Euler angles from time to time
unsigned long print_ms = 200, now_ms = 0, last_ms = 0; //print every print_ms

void setup() {

  // power up I2C port
#if defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2_TFT)
  pinMode(TFT_I2C_POWER, OUTPUT);
  digitalWrite(TFT_I2C_POWER, HIGH);
#endif

  Wire.begin();

  Serial.begin(115200);
  while (!Serial);  //wait for serial monitor to connect (can replace this with delay())

  if ( !ISM330.begin(0x6A) ) {  //0x6B is default with SFE library
    Serial.println("ISM330 not detected.");
    while (1) delay(1);
  }

  //  Serial.println("Applying settings.");

  ISM330.setDeviceConfig();
  ISM330.setBlockDataUpdate();

  // Set the output data rate and precision of the accelerometer, required even if accel is not used
  ISM330.setAccelDataRate(ISM_XL_ODR_104Hz);
  ISM330.setAccelFullScale(ISM_2g);


  // Set the output data rate and precision of the gyroscope
  ISM330.setGyroDataRate(ISM_GY_ODR_104Hz);
  ISM330.setGyroFullScale(ISM_500dps);

  // Turn on the gyroscope's filter and apply settings.
  ISM330.setGyroFilterLP1();
  ISM330.setGyroLP1Bandwidth(ISM_MEDIUM);

#ifdef CAL_GYRO

#define GYRO_SAMPLES 500
  Serial.println("Keep gyro still for offset calculation");
  delay(2000);


  int loopcount = 0;
  while (loopcount < GYRO_SAMPLES) {  //accumulate sums
    // Check if both gyroscope and accelerometer data are available.
    if ( ISM330.checkStatus() ) {
      ISM330.getGyro(&gyr);
      gxyz_offsets[0] += gyr.xData;
      gxyz_offsets[1] += gyr.yData;
      gxyz_offsets[2] += gyr.zData;
      loopcount++;
    }
  }

  // calculate and store gyro offsets

  for (int i = 0; i < 3; i++) gxyz_offsets[i] /= (float)loopcount;
#endif

  // reminder!
  Serial.print("Gyro offsets ");
  for (int i = 0; i < 3; i++) {
    Serial.print(gxyz_offsets[i], 1);
    Serial.print(" ");
  }
  Serial.println();

}  //end of setup()

// using SFE library,
// getAccel() returns milli_g, regardless of scale setting
// getGyro() returns milli_DPS, regardless

void loop() {

  static unsigned long last_us = 0, now_us = 0; //microsecond loop timing.

  // Check if both gyroscope and accelerometer data are available.
  if ( ISM330.checkStatus() ) {
    ISM330.getGyro(&gyr);
    now_us = micros();


    Gxyz[0] = (gyr.xData - gxyz_offsets[0]) * RadiansPerSecond;
    Gxyz[1] = (gyr.yData - gxyz_offsets[1]) * RadiansPerSecond;
    Gxyz[2] = (gyr.zData - gxyz_offsets[2]) * RadiansPerSecond;


    float deltat = (now_us - last_us) * 1.0e-6; //seconds since last update
    last_us = now_us;

    Mahony_integrate(Gxyz[0], Gxyz[1], Gxyz[2], deltat);

    // time to print?

    now_ms = millis(); //time to print?
    if (now_ms - last_ms >= print_ms) {
      last_ms = now_ms;

      // DEMO ANGLE CODE
      // Compute Tait-Bryan angles. >>>Strictly valid only for approximately level movement

      // In this coordinate system, the positive z-axis is up, X north, Y west.
      // Yaw is the angle between Sensor x-axis and Earth magnetic North
      // (or true North if corrected for local declination, looking down on the sensor
      // positive yaw is counterclockwise, which is not conventional for NED navigation.
      // Pitch is angle between sensor x-axis and Earth ground plane, toward the
      // Earth is positive, up toward the sky is negative. Roll is angle between
      // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
      // arise from the definition of the homogeneous rotation matrix constructed
      // from quaternions. Tait-Bryan angles as well as Euler angles are
      // non-commutative; that is, the get the correct orientation the rotations
      // must be applied in the correct order which for this configuration is yaw,
      // pitch, and then roll.
      // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
      // which has additional links.

      // WARNING: This angular conversion is for DEMONSTRATION PURPOSES ONLY. It WILL
      // MALFUNCTION for certain combinations of angles! See https://en.wikipedia.org/wiki/Gimbal_lock

      float roll  = atan2((q[0] * q[1] + q[2] * q[3]), 0.5 - (q[1] * q[1] + q[2] * q[2]));
      float pitch = -asin(2.0 * (q[0] * q[2] - q[1] * q[3]));  //pitch is positive, up
      //conventional yaw increases clockwise from North. Not that the MPU-6050 knows where North is.
      float yaw   = -atan2((q[1] * q[2] + q[0] * q[3]), 0.5 - (q[2] * q[2] + q[3] * q[3]));
      // to degrees
      yaw   *= 180.0 / PI;
      if (yaw < 0) yaw += 360.0; //compass circle
      pitch *= 180.0 / PI;
      roll *= 180.0 / PI;

      // print angles for serial plotter...
      //  Serial.print("ypr ");
      Serial.print(yaw, 0);
      Serial.print(", ");
      Serial.print(pitch, 0);
      Serial.print(", ");
      Serial.println(roll, 0);
    }
  }
}

// gyro-only integration

void Mahony_integrate(float gx, float gy, float gz, float deltat) {

  float qa, qb, qc;

  // Integrate rate of change of quaternion, q_dot = (1/2) q cross omega
  // https://www.ashwinnarayan.com/post/how-to-integrate-quaternions/
  deltat = 0.5 * deltat;
  gx *= deltat;   // pre-multiply omega by common time step
  gy *= deltat;
  gz *= deltat;
  qa = q[0];
  qb = q[1];
  qc = q[2];
  q[0] += (-qb * gx - qc * gy - q[3] * gz);
  q[1] += (qa * gx + qc * gz - q[3] * gy);
  q[2] += (qa * gy - qb * gz + q[3] * gx);
  q[3] += (qa * gz + qb * gy - qc * gx);

  // renormalise quaternion
  float recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  q[0] = q[0] * recipNorm;
  q[1] = q[1] * recipNorm;
  q[2] = q[2] * recipNorm;
  q[3] = q[3] * recipNorm;
}
