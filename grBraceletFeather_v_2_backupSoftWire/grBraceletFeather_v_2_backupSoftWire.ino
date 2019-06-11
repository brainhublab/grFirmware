#include <Arduino.h>
#include <SPI.h>
//#include <Wire.h>
//#include <SoftWire.h>
#include <I2C.h>

//#include <LSM6.h> //pololu accelerometer and gyro lib
//#include <LIS3MDL.h> //magnetometer lib
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <AsyncDelay.h>
#include <Vector.h>





//#include <ButtonEvents.h> //buttton lib --> https://github.com/fasteddy516/ButtonEvents

#include "OneButton.h"
#include "math.h"
#include <Battery.h>


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


#define IMUS_NUMBER 6 //number of   S
#define SENDING_DATA_TIMER_BOUND 6
#define PALM_INDEX 0


//objects

//LSM6 gyro_acc; //initialization of accelerometer and gyroscope vars
//LIS3MDL mag; //initialization of magnetometer


OneButton bttn(BUTTON_PIN, true);
Battery batt(3400, 4200, VBAT);

WiFiServer server(23);
WiFiClient client;
//WiFiUDP Udp;


unsigned long current_timer,  led_timer_rm, led_timer_lm, start_timer, end_timer; //timer needed for IR leds blinkin

bool finger_is_connected = false; //flag to verify if current finger IMU is connected
int8_t connected_imu_ids[IMUS_NUMBER] = {0, 0, 0, 0, 0, 0}; //boolean array conains values of connected IMU's
int8_t disconnected_imu_ids[IMUS_NUMBER] = {0, 0, 0, 0, 0, 0};

//const int IMU_SIGN[9] = {1, -1, -1, 1, -1, -1, 1, -1, -1};// IMU sign sets the palm IMU coordinate system to right coordinate system https://www.evl.uic.edu/ralph/508S98/coordinates.html
//const int IMU_SIGN[9] = { -1, 1, 1, 1, -1, -1, -1, 1, -1}; //TMP
//const int FINGER_IMU_SIGN[9] = { -1, 1, 1, 1, -1, -1, -1, 1, -1}; //IMU sign for fingers IMU's

const int IMU_SIGN[9] = {1, -1, -1, -1, 1, 1, 1, -1, -1}; //TMP
const int FINGER_IMU_SIGN[9] = { 1, -1, -1, -1, 1, 1, 1, -1, -1}; //IMU sign for fingers IMU's
IMU IMUS[IMUS_NUMBER]; //array from IMU for iterative call and read data

//char output_data[254];
//uint8_t output_data [20] = { bit(0) };
uint8_t output_data [116] = { bit(0) }; //TODO use it for byte array sendign optimisation


char imu_data[130] = {bit(0)};

//short battStatus = 0;

//led vars
int8_t brightness = 0;    // how bright the LED is
int8_t fade_amount = 40;    // how many points to fade the LED by
int8_t fade_coef = 20;
const short led_blink = 2000;
int8_t led_state = 0;

bool conn_flag = false;

//Battery vars
uint8_t old_batt_val = 0;
uint8_t cur_batt_val = 0;
bool IMU_powered_on = false;

//wifi vars
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

byte mac[6];                     // the MAC address of your Wifi shield

int led =  LED_BUILTIN;
bool getDataFlag = false;

String currentLine = "";

int status = WL_IDLE_STATUS;



////////////////////////////////////////

//int8_t sa0[6] = { PINKY, RING, MIDDLE, INDEX, THUMB, PALM};
unsigned long  time_now = 0;
unsigned long prev_time = 0;
bool chekyflag = true;

//SoftWire sw(SDA, SCL);
AsyncDelay samplingInterval;

//////////LIS3MDL




imuvector<int16_t> lis3mdlm; // magnetometer readings

uint8_t lis3mdllast_status; // status of last I2C transmission


lis3mdldeviceType _lis3mdldevice; // chip type
uint8_t lis3mdaddress;

uint16_t lis3mdlio_timeout = 5;
bool lis3mdldid_timeout;


///////////////LSM6

lsm6deviceType _lsm6device; // chip type
uint8_t lsm6address;

uint16_t lsm6io_timeout;
bool lsm6did_timeout;

int8_t* rxbuff;
int8_t* txbuff;
void setup()
{
  //pinMode(A1, OUTPUT);
  //digitalWrite(A1, LOW);

  //pinMode(20, INPUT); // remove output low
  //pinMode(21, INPUT); // remove output low
  //pinMode(20, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  // pinMode(21, INPUT_PULLUP);

  if (SERIAL_VERBOSE_MODE)
  {
    Serial.begin(115200);
  }
  //Serial.begin(115200);
  pinMode(LED, OUTPUT);
  // pinMode(BUTTON_PIN, INPUT_PULLUP);

  //setup wifi
  Serial.println("beforewifi");

  //initWifi();
  /* WiFi.setPins(8, 7, 4, 2);

    // check for the presence of the shield:
    if (WiFi.status() == WL_NO_SHIELD)
    {
     Serial.println("WiFi shield not present");
     // don't continue:
     while (true);
    }
    Serial.println("\nStarting connection to server...");
    //digitalWrite
    // if you get a connection, report back via serial:
    //  Udp.begin(23);
  */
  delay(2000);
  //sa0PinsInit();
  Serial.println("sa0init");


  //setup imus
  // i2cInit();
//  I2c.enablePullups();
  //I2c.setDelay_us(5);
  //I2c.setTimeout_ms(10);
  //I2c.setRxBuffer(rxbuff, 1);
  //I2c.setTxBuffer(txbuff, 1);
  I2c.begin();
  Serial.println("I2c init");

  sa0PinsInit();
  Serial.println("sa0init");

  for (int8_t i = 0; i < IMUS_NUMBER; i++)
  {
    digitalWrite(sa0[i], HIGH);
    checkIfIMUConnected(i);
    digitalWrite(sa0[i], LOW);
  }

  imuInit();
  Serial.println("Imu init");
  delay(100);
  analogWrite(LED, 0);
  delay(100);

  //calibrate();

  resetSa0();
  //powerOnIMU();
  //setup led
  led_timer_rm = 0;

  //button setup
  bttn.attachClick(bttnOnClick);
  bttn.attachDoubleClick(bttnOnDouble);
  bttn.attachLongPressStart(bttnOnLong);
  bttn.setDebounceTicks(20);

  //attachInterrupt(BUTTON_PIN, bttnTick, CHANGE);

  //settup battery
  batt.begin(3300, 2);
  for (int8_t i = 0; i < IMUS_NUMBER; i++)
  {
    digitalWrite(sa0[i], HIGH);
    checkIfIMUConnected(i);
    digitalWrite(sa0[i], LOW);
  }


  for (int8_t i = 0; i < 6; i++)
  {
    Serial.println(connected_imu_ids[i]);
  }


}

void loop()
{
  Serial.println("------------------------------------------------|");

  //  getData();
  for (int8_t i = 0; i < 6; i++)
  {
    // switchIMU(i);
    checkIfIMUConnected(i);
    Serial.println("checkIfConnected");
    switchIMU(i);
    Serial.println("switch");
    // checkIfIMUConnected(i);
    readIMU(i);
    Serial.println("read");
    // checkIfIMUConnected(i);
    /*    for (int8_t j = 0; j < 6; j++)
         {
          Serial.print("[ ");
          Serial.print(bitRead((sa0[i]), 3));
          Serial.print(" ]");
         }


         Serial.println("------------------------------------------------|");
    */
    if (i==5)
    {
      Serial.print("-|");
      Serial.print(i);
      Serial.print("|-");
      Serial.print(IMUS[i].mag_x);
      Serial.print(" ");
      Serial.print(IMUS[i].mag_y);
      Serial.print(" ");
      Serial.print(IMUS[i].mag_z);
      Serial.println(" ");
      /*
              Serial.print(IMUS[i].acc_x);
              Serial.print(" ");
              Serial.print(IMUS[i].acc_y);
              Serial.print(" ");
              Serial.println(IMUS[i].acc_z);
      */
    }

  }
  /*
      for (int8_t i = 0; i < 6; i++)
      {
        //switchIMU(i);
        //  checkIfIMUConnected(i);
        //Serial.println(connected_imu_ids[i]);
        Serial.print(connected_imu_ids[i]);
        Serial.print(" |---| ");
        Serial.print(disconnected_imu_ids[i]);
        Serial.print("  <--->  ");
      }
      Serial.println("=======================================");

  */
  //Serial.print(27,BYTE);   //Print "esc"


  /*
    if (status != WL_CONNECTED)
    {
      tryToConnect();
      Serial.println("\nStarting connection to server...");
    }
    else
    {
      WiFiClient client = server.available();   // listen for incoming clients
      if (client)
      {
        // if you get a client,
        String currentLine = "";// make a String to hold incoming data from the client
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

            // Check to see if the client request was "ga" - get attributes, "gd" - get data, "sd" - stop data
            if (currentLine.endsWith("ga")) {
              analogWrite(LED, 254);
              client.println("GR[L] 123456" );
              analogWrite(LED, 0);

            }
            if (currentLine.endsWith("gd")) {
              analogWrite(LED, 0);
              getDataFlag = true;
            }
            if (currentLine.endsWith("sd")) {
              digitalWrite(LED, LOW);
              Serial.println("Stop Reading");
              getDataFlag = false;
            }
          }
          if (getDataFlag)
          {
            getData();
            resetSa0();

            generatePackage();
            //client.println(output_data);
            client.write(output_data, sizeof(output_data));
          }

          end_timer = millis();
          unsigned long  tmp = end_timer - start_timer;
          Serial.print("------------------- ");
          Serial.println(tmp);
          Serial.println();
          for (int8_t i = 0; i < 6; i++)
          {
            //Serial.println(connected_imu_ids[i]);
            Serial.print(connected_imu_ids[i]);
            Serial.print(" |---| ");
            Serial.print(disconnected_imu_ids[i]);
            Serial.print("  <--->  ");
          }
          Serial.println("=======================================");

        }
        // close the connection:
        client.stop();
        Serial.println("client disconnected");
      }
    }
  */

}
