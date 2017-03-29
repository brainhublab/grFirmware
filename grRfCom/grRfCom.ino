/*include arduino libs*/
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

/*include gattAttributes header with gatt methods and structs */
#include "sensorAttributes.h"

/*including compas acclerometer and gyro */
#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>

/* define IMU sensors */
LIS3MDL compass; //TODO need to be mag
LSM6 gyro;


//sensor sign
//int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1};

//containers for data report TODO  move t to struct
//char report_gyro[20];
//char report_magnet[20];
//char report_accelerometer[20];

//Some defines for gyroscope data
//#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer

//#define ToRad(x) ((x)*0.01745329252)  // *pi/180
//#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

#define TCAADDR 0x70

// LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 or LIS3MDL library to find the right values for your board

//#define M_X_MIN -1000
//#define M_Y_MIN -1000
//#define M_Z_MIN -1000
//#define M_X_MAX +1000
//#define M_Y_MAX +1000
//#define M_Z_MAX +1000



// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// Select port on multiplexer
void tca_select(uint8_t i) {
  if(i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}


sensor allSensors[6];


void init_sensors(sensor arr[]) {
  for(uint8_t i = 0; i < 6; i++) {
    tca_select(i);
    
    arr[i].gyro.init();
    arr[i].gyro.enableDefault();

    arr[i].compass.init();
    arr[i].compass.enableDefault();
  }
}

void read_finger(uint8_t port, sensor *s) {
  char report_gyro[20];
  char report_magnet[20];
  char report_accelerometer[20];
  
  tca_select(port);
  delay(20);

  //Serial.print("swithc to port: "); Serial.println(port);

  s->compass.read();
  s->gyro.readGyro();
  s->gyro.readAcc();

  snprintf(report_gyro, sizeof(report_gyro), "%d %d %d",
    s->gyro.g.x, s->gyro.g.y, s->gyro.g.z);
  //Serial.print(port); Serial.print(" - ");
  Serial.println(report_gyro);
  
  snprintf(report_accelerometer, sizeof(report_accelerometer), "%d %d %d",
    s->gyro.a.x, s->gyro.a.y, s->gyro.a.z);
  //Serial.print(port); Serial.print(" - ");
  Serial.println(report_accelerometer);
  
  snprintf(report_magnet, sizeof(report_magnet), "%d %d %d",
    s->compass.m.x, s->compass.m.y, s->compass.m.z);
  //Serial.print(port); Serial.print(" - ");
  Serial.println(report_magnet);
   
}

void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);
  Serial.begin(9600);

  randomSeed(micros());

  //init compas data
  Wire.begin();
  //Wire.setClock(160);

   init_sensors(allSensors);

   Serial.println(F("******************************"));

   Serial.println(F("******************************"));

}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/

void loop(void)
{
  
  for(uint8_t i = 0; i < 6; i++) {
    read_finger(i, &allSensors[i]);
    Serial.println(i);
    delay(200);  
  }
}
