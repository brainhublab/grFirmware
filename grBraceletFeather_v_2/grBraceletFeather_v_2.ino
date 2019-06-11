#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LSM6.h> //pololu accelerometer and gyro lib
#include <LIS3MDL.h> //magnetometer lib
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <Adafruit_SleepyDog.h>
//#include "LowPower.h"
//#include "avdweb_VirtualDelay.h"
//#include <AsyncDelay.h>
#include "avdweb_Switch.h"

//#include "OneButton.h"
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


#define IMUS_NUMBER 6 //number of   S
#define SENDING_DATA_TIMER_BOUND 6
#define PALM_INDEX 0


//objects

LSM6 gyro_acc; //initialization of accelerometer and gyroscope vars
LIS3MDL mag; //initialization of magnetometer

WiFiServer server(23);
WiFiClient client;
//WiFiUDP Udp;


unsigned long current_timer,  led_timer_rm, led_timer_lm, start_timer, end_timer, onSessionTimer, oldOnSessionTimer; //timer needed for IR leds blinkin

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
uint8_t output_data [117] = { bit(0) }; //TODO use it for byte array sendign optimisation


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
uint8_t currentBatteryLevel = 0;
bool IMU_powered_on = false;
unsigned long battTimer;
uint16_t battMeasurePeriod = 60000;
uint16_t maxBatteryVoltage = 4160;
uint16_t minBatteryVoltage = 3000;


//wifi vars
char ssid[] = SECRET_SSID;        // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

byte mac[6];                     // the MAC address of your Wifi shield

int led =  LED_BUILTIN;
bool getDataFlag = false;

String currentLine = "";

int status = WL_IDLE_STATUS;

//AsyncDelay delay_200ms;
//AsyncDelay delay_20ms;

Switch bttn = Switch(BUTTON_PIN);
//OneButton buttn(BUTTON_PIN, true);
bool isLEDOn = false;

bool calibrationFlag = false;

bool powerSaveMode = false;

//WiFiClient client;

void setup()
{
  //pinMode(A1, OUTPUT);
  //digitalWrite(A1, LOW);

#ifdef USBCON
  USBDevice.attach();
#endif

  //pinMode(20, INPUT_PULLUP); // and make SDA high i.e. send I2C STOP control.
  //pinMode(21, INPUT_PULLUP);

  if (SERIAL_VERBOSE_MODE)
  {
    Serial.begin(115200);
  }
  //Serial.begin(115200);
  pinMode(LED, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);

  //setup wifi
  initWifi();
  delay(2000);
  //sa0PinsInit();


  //setup imus
  i2cInit();
  sa0PinsInit();
  for (int8_t i = 0; i < IMUS_NUMBER; i++)
  {
    digitalWrite(sa0[i], HIGH);
    checkIfIMUConnected(i);
    digitalWrite(sa0[i], LOW);
  }

  imuInit();
  calibrate();

  resetSa0();
  //powerOnIMU();
  led_timer_rm = 0;
  startTimer(40);

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

  oldOnSessionTimer = 0;

  //battery setup
  currentBatteryLevel = getBattLevel();
 /* attachInterrupt(BUTTON_PIN, buttonClick, CHANGE);
  buttn.attachClick(click1);
  buttn.attachDoubleClick(doubleclick1);
  buttn.attachLongPressStart(longPress1);
  */
}

void loop()
{
  //bttnTick();
  // buttonClick();
  if (millis() - battTimer >= battMeasurePeriod)
  {
    battTimer = millis();
    updatePowerMode();
    ledBlink();
  }

  /*
    //  getData();
    for (int8_t i = 0; i < 6; i++)
    {
      // switchIMU(i);
      checkIfIMUConnected(i);
      switchIMU(i);
      // checkIfIMUConnected(i);
      readIMU(i);
      // checkIfIMUConnected(i);

        if (i == 5)
        {
        Serial.print("-|");
        Serial.print(i);
        Serial.print("|-");
        Serial.print(IMUS[i].gyro_x);
        Serial.print(" ");
        Serial.print(IMUS[i].gyro_y);
        Serial.print(" ");
        Serial.print(IMUS[i].gyro_z);
        Serial.print(" ");

        Serial.print(IMUS[i].acc_x);
        Serial.print(" ");
        Serial.print(IMUS[i].acc_y);
        Serial.print(" ");
        Serial.println(IMUS[i].acc_z);
        }

    }*/
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

  // Serial.print("------------------------------------------------------------------------");
  //Serial.print(getBattVoltage());
  // Serial.println((int8_t)getBattLevel());

  if (status != WL_CONNECTED)
  {
    tryToConnect();
    Serial.println("\nStarting connection to server...");
  }
  else
  {
    buttonClick();
    //WiFiClient
     client = server.available();   // listen for incoming clients
    // Serial.println("---------------------------------------CLIENT WAIT");
    if (client)
    {
      // if you get a client,
      String currentLine = "";// make a String to hold incoming data from the client
      while (client.connected())
      {
        buttonClick();
        //Serial.println("------------------------------------------CLIENT CONNECTED");

        if (millis() - battTimer >= battMeasurePeriod)
        {
          battTimer = millis();
          currentBatteryLevel = getBattLevel();
        }
        //Serial.println("------------------------------------------CLIENT CONNECTED");


        // loop while the client's connected
        checkIncomingEvent(&client);
        onSessionTimer = millis();
        //start_timer = onSessionTimer;
        if (getDataFlag && onSessionTimer - oldOnSessionTimer >= 25)
        {
          // Serial.print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>____");
          // Serial.println(onSessionTimer - oldOnSessionTimer);
          oldOnSessionTimer = onSessionTimer;
          getData();
          resetSa0();

          generatePackage();
          //client.println(output_data);
          client.write(output_data, sizeof(output_data));
          //    Serial.print("|---");
          
           /* Serial.print(IMUS[5].gyro_x);
            Serial.print(" ");
            Serial.print(IMUS[5].gyro_y);
            Serial.print(" ");
            Serial.print(IMUS[5].gyro_z);
            Serial.print(" ");
            */
            Serial.print(IMUS[5].acc_x);
            Serial.print(" ");
            Serial.print(IMUS[5].acc_y);
            Serial.print(" ");
            Serial.print(IMUS[5].acc_z);
            Serial.print(" ");
            Serial.println();
          
          /* end_timer = millis();
            unsigned long  tmp = end_timer - start_timer;
            Serial.print("------------------- ");
            Serial.println(tmp);
            Serial.println();

          */
          /*
                    for (int8_t i = 0; i < 6; i++)
                    {
                      //Serial.println(connected_imu_ids[i]);
                      Serial.print(connected_imu_ids[i]);
                      Serial.print(" |---| ");
                      Serial.print(disconnected_imu_ids[i]);
                      Serial.print("  <--->  ");
                    }
                    Serial.println("=======================================");
          */

        }


      }
      // close the connection:
      client.stop();
      Serial.println("client disconnected");
    }
  }


}
