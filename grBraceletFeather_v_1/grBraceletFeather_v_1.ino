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


#include "config.h"
#include "imu.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif
int asd = 0;

#define hb(x) ( (x) >> (8) & (0xff) ) // keep upper 8 bits
#define lb(x) ( (x) & (0xff) ) // keep lower 8 bits

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

#define PALM_INDEX 5

unsigned long current_timer, conn_timer, temporary_timer, batt_timer, led_timer_rm, led_timer_lm, start_timer, end_timer; //timer needed for IR leds blinkin

bool finger_is_connected = false; //flag to verify if current finger IMU is connected
int8_t connected_imu_ids[IMUS_NUMBER] = {0, 0, 0, 0, 0, 0}; //boolean array conains values of connected IMU's

const int IMU_SIGN[9] = {1, -1, -1, 1, -1, -1, 1, -1, -1};// IMU sign sets the palm IMU coordinate system to right coordinate system https://www.evl.uic.edu/ralph/508S98/coordinates.html

const int FINGER_IMU_SIGN[9] = { -1, 1, 1, 1, -1, -1, -1, 1, -1}; //IMU sign for fingers IMU's
IMU IMUS[IMUS_NUMBER]; //array from IMU for iterative call and read data

//char output_data[20];
uint8_t output_data [20] = { bit(0) };

uint8_t packet_data[19] = {bit(0)};



//short battStatus = 0;

byte brightness = 0;    // how bright the LED is
byte fade_amount = 40;    // how many points to fade the LED by
byte fade_coef = 20;
const short led_blink = 2000;
byte led_state = 0;

volatile byte state = LOW;

bool conn_flag = false;
int8_t transmission = 4;
bool transmission_flag  = false;

//byte old_batt_p = 0;
uint8_t old_batt_val = 0;
uint8_t cur_batt_val = 0;
bool IMU_powered_on = false;


///////////
//Madgwick

unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;


float ax, ay, az;
float gx, gy, gz;
int16_t roll, pitch, yaw, last_palm_yaw;

float hui  = 0.0f;

////////////////////////////////////////

bool sign_arr[8] = {1, 1, 1, 1, 1, 1, 1, 1};


void setup()
{
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

  for (byte i = 0; i < IMUS_NUMBER; i++)
  {
    IMUS[i].m_filter.begin(25);
  }


  // initialize variables to pace updates to correct rate
  microsPerReading = 1000000 / 4;
  microsPrevious = micros();

  transmission =  SOH;
}

void loop()
{


  buttonClick();
  while ( ble.available() )
  {
    //uint8_t c =
    int8_t c = (int8_t)ble.read();
    if (c == SOH || c == EOT)
    {
      transmission = c;
    }
    if (c == BAT)
    {
      Serial.println("Battery check is trigered");
      battery.update(getBattPercents());

    }

  }


  current_timer = millis();

  if (true)//(BLEConnected())
  {
    /*
      if (!IMU_powered_on)
      {
      powerOnIMU();
      fade_coef = 20;
      }
    */
    if (transmission == SOH) //conn_flag
    {


      //   ledBlink();

      transmission_flag = true;



      //grPrint("================================");
      // 17 - 18 ms


      start_timer = millis();



      for (int8_t i = 0; i < IMUS_NUMBER; i++)
      {
        //int8_t i = 5;



        //  grPrint("================================");

        // 0 ms
        switchIMU(i);


        // 3 ms
        readIMU(i);

        ax = convertRawAcceleration(IMUS[i].acc_x);
        ay = convertRawAcceleration(IMUS[i].acc_y);
        az = convertRawAcceleration(IMUS[i].acc_z);

        gx = convertRawGyro(IMUS[i].gyro_x);
        gy = convertRawGyro(IMUS[i].gyro_y);
        gz = convertRawGyro(IMUS[i].gyro_z);

        IMUS[i].m_filter.updateIMU(gx, gy, gz, ax, ay, az);

        roll = (int16_t)IMUS[i].m_filter.getRoll();
        pitch = (int16_t)IMUS[i].m_filter.getPitch();
        yaw = (int16_t)IMUS[i].m_filter.getYaw();




/*
        Serial.print("Orientation: ");
        Serial.print(yaw);
        Serial.print(" ");
        Serial.print(pitch);
        Serial.print(" ");
        Serial.println(roll);
        // Serial.print(" ");
        // Serial.println(bitTo);
*/
        uint8_t palm_acc_z_low;


        if (i == 0)
        {
          last_palm_yaw = yaw;


          packet_data[7] = hb(yaw);
          packet_data[8] = lb(yaw);

          packet_data[9] = hb(pitch);
          packet_data[10] = lb(pitch);

          packet_data[11] = hb(roll);
          packet_data[12] = lb(roll);

          packet_data[13] = hb(IMUS[i].acc_x);
          packet_data[14] = lb(IMUS[i].acc_x);
          packet_data[15] = hb(IMUS[i].acc_y);
          packet_data[16] = lb(IMUS[i].acc_y);
          packet_data[17] = hb(IMUS[i].acc_z);
          packet_data[18] = lb(IMUS[i].acc_z);




        }
        else
        {
          // uint8_t uroll = (uint8_t) (roll < 0 ? -roll : roll);
          // uint8_t upitch = (uint8_t) (pitch < 0 ? -pitch : pitch);
          uint8_t relative_yaw =  yaw - last_palm_yaw;
          uint8_t uyaw = (uint8_t) (relative_yaw < 0 ? -relative_yaw : relative_yaw);
          sign_arr[i] = (relative_yaw > 0);

          packet_data[i + 1] = uyaw;

          if (i == 5) {
            packet_data[0] = 1; // from 0 to 255
            packet_data[1] = bitArrayToInt8(sign_arr, 8);
            // send
           // gatt.setChar(sensorServiceId, packet_data, 19);

            Serial.println("Data is Sended");
            end_timer = millis();
            unsigned long  tmp = end_timer - start_timer;
            Serial.print(" ");
            Serial.println(tmp);
          }


        }

        /*
                  Serial.print("Orientation: ");
                  Serial.print(yaw);
                  Serial.print(" ");
                  Serial.print(pitch);
                  Serial.print(" ");
                  Serial.println(roll);*/
        //ble.write(output_data, 20);
        //ble.flush();// delay(5);
        //gatt.setChar(sensorServiceId, output_data, 20);



        // Oms
        //generatePackage(i);
        //ble.write(output_data, 20);
        // ble.flush();// delay(5);



        // 10 ms // TODO: in some cases - 40ms

        //delay(27);


        // EOF for loop
      }





    }
    else
    {
      if (transmission == EOT && transmission_flag)
      {
        // conn_flag = false;
        transmission_flag = false;
        // powerOffIMU();
        grPrint("trigered disable transmission!");
        // digitalWrite(LED, LOW);
        analogWrite(LED, 0);
        // fade_coef = 20;

      }
    }

  }
  else
  {

    if (conn_flag)
    {
      conn_flag = false;
      //   powerOffIMU();
      //fade_coef = 20;
      grPrint("disconnected11!");
    }




  }
}

float convertRawAcceleration(int aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  //return (float)aRaw;
  float a = (aRaw * 8.0) / 65535;
  return a;
}

float convertRawGyro(int gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  //return (float)gRaw;

  float g = (gRaw * 4000.0) / 65535;
  return g;
}

int8_t bitArrayToInt8(bool arr[], int count)
{
  int ret = 0;
  int tmp;
  for (int i = 0; i < count; i++) {
    tmp = arr[i];
    ret |= tmp << (count - i - 1);
  }
  return ret;
}
