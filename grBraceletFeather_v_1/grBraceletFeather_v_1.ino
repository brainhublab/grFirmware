#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LSM6.h> //pololu accelerometer and gyro lib
#include <LIS3MDL.h> //magnetometer lib


//#include <ButtonEvents.h> //buttton lib --> https://github.com/fasteddy516/ButtonEvents
#include "avdweb_Switch.h"

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEBattery.h"
#include "Adafruit_BLEGatt.h"
#include "avdweb_Switch.h"
#include "MadgwickAHRS.h"
#include "math.h"


#include "config.h"
#include "imu.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif
int asd = 0;

#define hb(x) ( (x) >> (8) & (0xff) ) // keep upper 8 bits
#define lb(x) ( (x) & (0xff) ) // keep lower 8 bits

#define convertRawAcceleration(accRaw) ((accRaw * 0.8) / 65535)
#define convertRawGyro(gyroRaw) ((gyroRaw * 4000.0) / 65535)

/*
  float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  //return (float)aRaw;
  return (aRaw * 8.0) / 65535;
  }
*/
/*
  float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  //return (float)gRaw;
  return (gRaw * 4000.0) / 65535;
  }
*/

//objects
/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BLEBattery battery(ble);
Adafruit_BLEGatt gatt(ble);

//prepare services
int8_t sensorServiceId;
uint8_t sensorServiceUUID[] {0xfc, 0xed, 0x64, 0x08, 0xc0, 0x15, 0x45, 0xea, 0xb5, 0x0d, 0x1d, 0xe3, 0x2a, 0x1c, 0x2f, 0x6d};

LSM6 gyro_acc; //initialization of accelerometer and gyroscope vars
LIS3MDL mag; //initialization of magnetometer

Switch bttn = Switch(BUTTON_PIN);
//ButtonEvents bttn;
//////////////////////////////////////////////////
#define IMUS_NUMBER 6 //number of   S
#define SENDING_DATA_TIMER_BOUND 6

#define PALM_INDEX 0

unsigned long current_timer, conn_timer, temporary_timer, batt_timer, led_timer_rm, led_timer_lm, start_timer, end_timer; //timer needed for IR leds blinkin

bool finger_is_connected = false; //flag to verify if current finger IMU is connected
int8_t connected_imu_ids[IMUS_NUMBER] = {0, 0, 0, 0, 0, 0}; //boolean array conains values of connected IMU's

//const int IMU_SIGN[9] = {1, -1, -1, 1, -1, -1, 1, -1, -1};// IMU sign sets the palm IMU coordinate system to right coordinate system https://www.evl.uic.edu/ralph/508S98/coordinates.html
//const int IMU_SIGN[9] = { -1, 1, 1, 1, -1, -1, -1, 1, -1}; //TMP
//const int FINGER_IMU_SIGN[9] = { -1, 1, 1, 1, -1, -1, -1, 1, -1}; //IMU sign for fingers IMU's

const int IMU_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //TMP
const int FINGER_IMU_SIGN[9] = { 1,-1,-1,-1,1,1,1,-1,-1}; //IMU sign for fingers IMU's
IMU IMUS[IMUS_NUMBER]; //array from IMU for iterative call and read data

//char output_data[20];
//uint8_t output_data [20] = { bit(0) };
uint8_t output_data [108] = { bit(0) };

uint8_t packet_data[19] = {bit(0)};



//short battStatus = 0;

int8_t brightness = 0;    // how bright the LED is
int8_t fade_amount = 40;    // how many points to fade the LED by
int8_t fade_coef = 20;
const short led_blink = 2000;
int8_t led_state = 0;

volatile int8_t state = LOW;

bool conn_flag = false;
int8_t transmission = 4;
bool transmission_flag  = false;

//int8_t old_batt_p = 0;
uint8_t old_batt_val = 0;
uint8_t cur_batt_val = 0;
bool IMU_powered_on = false;


///////////
//Madgwick

unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;


float ax, ay, az;
float gx, gy, gz;
int16_t roll, pitch, yaw, last_palm_yaw, last_palm_pitch, last_palm_roll;

//float roll, pitch, yaw, last_palm_yaw, last_palm_pitch, last_palm_roll;

float hui  = 0.0f;

////////////////////////////////////////

bool sign_arr[8] = {1, 1, 1, 1, 1, 1, 1, 1};


//some angle vars;
float angle = 0.0f;
float K = 0.0f;

void setup()
{
  pinMode(A1, OUTPUT);
    digitalWrite(A1, LOW);

  if (SERIAL_VERBOSE_MODE)
  {
    Serial.begin(115200);
  }
  pinMode(LED, OUTPUT);
  // pinMode(INDEX, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  initBLE();

  // TODO: ble.setInterCharWriteDelay(2); // 5 ms

  battery.begin(true);
  delay(200);

  i2cInit();
  sa0PinsInit();
  //BLE init need to be at the end
  imuInit();
  delay(100);
  analogWrite(LED, 0);
  delay(100);

  calibrate();
  if (!BLEConnected())
  {
    //powerOffIMU();
  }
  resetSa0();

  led_timer_rm = 0;

  battery.update((uint8_t)getBattPercents());

  for (int8_t i = 0; i < IMUS_NUMBER; i++)
  {
    if (i == 0 || i == 4)
    {
      IMUS[i].m_filter.begin(47);

    }
    //checkIfIMUConnected(i);
  }


  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 4;
  microsPrevious = micros();


}

void loop()
{
  start_timer = millis();

  //buttonClick();
  //bleRead();
  // current_timer = millis();

  // 17 - 18 m
  for (int8_t i = 0; i < IMUS_NUMBER; i++)
  {
    //int8_t i = 5;


   // if (connected_imu_ids[i])
    //{
      switchIMU(i);
      // 3 ms
      readIMU(i);
      updateFilter(i);
      updateAngles(i);
      // Oms
     //generatePackage(i);
   
      //ble.write(output_data, 20);
      //ble.flush();
      //delay(3000);
   // }

  }
  genPack();
  ble.write(output_data, 108);
  //ble.flush();// delay(5);
 // gatt.setChar(sensorServiceId, output_data, 108);
  end_timer = millis();
  unsigned long  tmp = end_timer - start_timer;

  Serial.print("------------------- ");
  Serial.println(tmp);
  Serial.println();
}
