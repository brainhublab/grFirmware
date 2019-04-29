#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LSM6.h> //pololu accelerometer and gyro lib
#include <LIS3MDL.h> //magnetometer lib
#include <WiFi101.h>


//#include <ButtonEvents.h> //buttton lib --> https://github.com/fasteddy516/ButtonEvents
#include "avdweb_Switch.h"

#include "avdweb_Switch.h"
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

const int IMU_SIGN[9] = {1, -1, -1, -1, 1, 1, 1, -1, -1}; //TMP
const int FINGER_IMU_SIGN[9] = { 1, -1, -1, -1, 1, 1, 1, -1, -1}; //IMU sign for fingers IMU's
IMU IMUS[IMUS_NUMBER]; //array from IMU for iterative call and read data

//char output_data[20];
//uint8_t output_data [20] = { bit(0) };
char output_data [120] = { bit(0) };

char imu_data[130] = {bit(0)};



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

//wifi vars
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

byte mac[6];                     // the MAC address of your Wifi shield

int led =  LED_BUILTIN;
bool getData = false;

String currentLine = "";

int status = WL_IDLE_STATUS;

WiFiServer server(23);
WiFiClient client;

////////////////////////////////////////

bool sign_arr[8] = {1, 1, 1, 1, 1, 1, 1, 1};

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

  initWifi();


  delay(200);

  // i2cInit();
  // sa0PinsInit();
  //BLE init need to be at the end
  // imuInit();
  delay(100);
  analogWrite(LED, 0);
  delay(100);

  // calibrate();

  // resetSa0();

  led_timer_rm = 0;


}

void loop()
{

  WiFiClient client = server.available();   // listen for incoming clients


  if (client)
  { // if you get a client,
    Serial.println("new client");           // print a message out the serial port
    String currentLine = "";
    int i = 0;// make a String to hold incoming data from the client

    while (client.connected())
    {
      unsigned long start_timer = millis();
      
      // loop while the client's connected
      if (client.available())
      { // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        if (c == '\n') {                    // if the byte is a newline character
          if (currentLine.length() == 0)
          {
            break;
          }
          else
          { // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        }
        else if (c != '\r')
        { // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }

        // Check to see if the client request was "GET /H" or "GET /L":
        if (currentLine.endsWith("ga")) {
          digitalWrite(led, HIGH);
          client.println("GR[L] 123456" );
          digitalWrite(led, LOW);

        }
        if (currentLine.endsWith("gd")) {
          digitalWrite(led, LOW);
          getData = true;
        }
        if (currentLine.endsWith("sd")) {
          digitalWrite(led, LOW);
          Serial.println("Stop Reading");
          getData = false;
        }
      }
      if (getData)
      {
        //getting data
        /* memset(output_data, 0, sizeof(output_data));
          snprintf(output_data, sizeof(output_data), "%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d",
                  1, 2, 3, 4, 5, 6, 7, 8, 9,
                  1, 2, 3, 4, 5, 6, 7, 8, 9,
                  1, 2, 3, 4, 5, 6, 7, 8, 9,
                  1, 2, 3, 4, 5, 6, 7, 8, 9,
                  1, 2, 3, 4, 5, 6, 7, 8, 9,
                  1, 2, 3, 4, 5, 6, 7, 8, 9, i);

          client.println(output_data);
          //delay(1000);
          i++;*/
        generatePackage(0);
        long sz = 0;
        //output_data[0]=i;
        sz = client.println(output_data);
        //delay(1000);
        // client.write(imu_data);
         Serial.println(output_data);
        i++;
        delay(8);
      }

      end_timer = millis();
      unsigned long  tmp = end_timer - start_timer;

      Serial.print("------------------- ");
      Serial.println(tmp);
      Serial.println();

    }
    // close the connection:
    client.stop();
    Serial.println("client disconnected");
  }


  /*
    start_timer = millis();

    //buttonClick();

    for (int8_t i = 0; i < IMUS_NUMBER; i++)
    {
      //int8_t i = 5;


      // if (connected_imu_ids[i])
      //{
      switchIMU(i);
      // 3 ms
      readIMU(i);


    }
    genPack();


  */
}
