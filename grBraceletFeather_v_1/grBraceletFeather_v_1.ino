#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "config.h"
#include "sensor.h"

#include <LSM6.h> //pololu accelerometer and gyro lib
#include <LIS3MDL.h> //magnetometer lib
#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif
  int asd = 0;

//objects 
/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

//////////////////////////////////////////////////
#define IMUS_N 6 //number of IMUS
#define SENDING_DATA_TIMER_BOUND 6

#define PALM_INDEX 5

unsigned long current_timer, last_timer, temporary_timer, buffer_timer; //timer needed for IR leds blinkin

bool finger_is_connected = false; //flag to verify if current finger IMU is connected
bool connected_fingers_id[IMUS_N] = {0, 0, 0, 0, 0, 0}; //boolean array conains values of connected IMU's

const int IMU_SIGN[9] = {1, -1, -1, 1, -1, -1, 1, -1, -1};// IMU sign sets the palm IMU coordinate system to right coordinate system https://www.evl.uic.edu/ralph/508S98/coordinates.html

const int FINGER_IMU_SIGN[9] = { -1, 1, 1, 1, -1, -1, -1, 1, -1}; //IMU sign for fingers IMU's
IMU IMUS[IMUS_N]; //array from IMU for iterative call and read data

////////////////////////////////////////
LSM6 gyro_acc; //initialization of accelerometer and gyroscope vars
LIS3MDL mag; //initialization of magnetometer
void setup()
{
  if(SERIAL_VERBOSE_MODE)
  {
    Serial.begin(115200);
  }
  i2cPinsInit();
  i2cInit();
  delay(200);
  //BLE init need to be at the end 
  initBLE();
  digitalWrite(PALM, HIGH);

  gyro_acc.init(gyro_acc.device_auto, gyro_acc.sa0_high);
  gyro_acc.enableDefault();

  
  
}

void loop()
{
  //if(BLEConnected())
  //{
   // grPrint("Work!");
    //delay(200);
    Serial.println("--------------------------");
    if(asd > 20)
    {
      digitalWrite(PALM, LOW);
    }
    else
    {
      digitalWrite(PALM, HIGH);
    }
    gyro_acc.readGyro();
    Serial.println(getBatVoltage());
   
    
    asd++;
  //}
 // else
  //{
  //  grPrint("disconnected!");
  
 // }
  
}

