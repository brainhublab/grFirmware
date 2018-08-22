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
#include "avdweb_Switch.h"

#include "config.h"
#include "imu.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif
int asd = 0;

//objects
/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BLEBattery battery(ble);

LSM6 gyro_acc; //initialization of accelerometer and gyroscope vars
LIS3MDL mag; //initialization of magnetometer

Switch bttn = Switch(BUTTON_PIN);
//ButtonEvents bttn;
//////////////////////////////////////////////////
#define IMUS_NUMBER 6 //number of IMUS
#define SENDING_DATA_TIMER_BOUND 6

#define PALM_INDEX 5

unsigned long current_timer, conn_timer, temporary_timer, buffer_timer, led_timer_rm, led_timer_lm; //timer needed for IR leds blinkin

bool finger_is_connected = false; //flag to verify if current finger IMU is connected
byte connected_imu_ids[IMUS_NUMBER] = {0, 0, 0, 0, 0, 0}; //boolean array conains values of connected IMU's

const int IMU_SIGN[9] = {1, -1, -1, 1, -1, -1, 1, -1, -1};// IMU sign sets the palm IMU coordinate system to right coordinate system https://www.evl.uic.edu/ralph/508S98/coordinates.html

const int FINGER_IMU_SIGN[9] = { -1, 1, 1, 1, -1, -1, -1, 1, -1}; //IMU sign for fingers IMU's
IMU IMUS[IMUS_NUMBER]; //array from IMU for iterative call and read data

char output_data[20];

short battStatus = 0;

byte brightness = 0;    // how bright the LED is
byte fade_amount = 5;    // how many points to fade the LED by
byte fade_coef = 40;
const short led_blink = 2000;
byte led_state = 0;

volatile byte state = LOW;

bool conn_flag = false;

////////////////////////////////////////

void setup()
{
  if (SERIAL_VERBOSE_MODE)
  {
    Serial.begin(115200);
  }
  pinMode(LED, OUTPUT);

  initBLE();
  battery.begin(true);
  delay(200);

  i2cInit();
  sa0PinsInit();
  //BLE init need to be at the end
  imuInit();
  delay(20);

  calibrate();
  led_timer_rm = 0;
}

void loop()
{
  current_timer = millis();
  ledBlink();
  buttonClick();
  if ((current_timer - conn_timer >= 500) || conn_flag)
  {
    conn_timer = current_timer;
    if (BLEConnected())
    {
      grPrint("================================");
      Serial.println(int(getBattPercents()));

      battery.update(int(getBattPercents()));
     // ble.print(battStatus);
      for (byte i = 0; i < IMUS_NUMBER; i++)
      {
        if (connected_imu_ids[i])
        {
          switchIMU(i);
          readIMU();
          resetSa0();
          /*
                  Serial.print(i);
                  Serial.print(" ");
                  Serial.print(IMUS[i].acc_x);
                  Serial.print(" ");
                  Serial.print(IMUS[i].acc_y);
                  Serial.print(" ");
                  Serial.print(IMUS[i].acc_z);
                  Serial.print("   batt: ");
                  Serial.print(battStatus);
                  Serial.print("\n");

          */
          generatePackage(i);
          ble.print(output_data);
        }
      }
    }
    else
    {
      grPrint("disconnected!");
      //Serial.println(battStatus);
      conn_flag = false;

    }

  }
}

