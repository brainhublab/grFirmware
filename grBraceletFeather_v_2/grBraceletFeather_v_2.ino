#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <LSM6.h> //pololu accelerometer and gyro lib
#include <LIS3MDL.h> //magnetometer lib
#include <WiFi101.h>
#include <WiFiUdp.h>
#include <avdweb_Switch.h>

#include "math.h"
#include "config.h"
#include "imu.h"

#if SOFTWARE_SERIAL_AVAILABLE
#include <SoftwareSerial.h>
#endif

// Watchdog to reset controller if I2C hangs
#include <Adafruit_SleepyDog.h>

unsigned long oldTime;

// Set watchdog timeouts
#define WATCHDOG_SETUP_TIMEOUT 4000
#define WATCHDOG_LOOP_TIMEOUT 500

// Used for the interrupt timer
#define CPU_HZ 48000000
#define TIMER_PRESCALER_DIV 1024

#define hb(x) ( (x) >> (8) & (0xff) ) // keep upper 8 bits
#define lb(x) ( (x) & (0xff) ) // keep lower 8 bits

#define convertRawAcceleration(accRaw) ((accRaw * 0.8) / 65535)
#define convertRawGyro(gyroRaw) ((gyroRaw * 4000.0) / 65535)

/*
   float convertRawAcceleration(int aRaw) {
     since we are using 2G range
     -2g maps to a raw value of -32768
     +2g maps to a raw value of 32767
     return (float)aRaw;
     return (aRaw * 8.0) / 65535;
   }
*/

/*
   float convertRawGyro(int gRaw) {
     since we are using 250 degrees/seconds range
     -250 maps to a raw value of -32768
     +250 maps to a raw value of 32767
     return (float)gRaw;
     return (gRaw * 4000.0) / 65535;
   }
*/

#define IMUS_NUMBER 6 //number of IMUS
#define SENDING_DATA_TIMER_BOUND 6
#define PALM_INDEX 0

// flags for button handling
bool SINGLE_CLICK = false;
bool DOUBLE_CLICK = false;
bool LONG_PRESS = false;

//objects
LSM6 gyro_acc; //initialization of accelerometer and gyroscope vars
LIS3MDL mag; //initialization of magnetometer

WiFiServer server(23);
WiFiClient client;
//WiFiUDP Udp;


unsigned long current_timer,  led_timer_rm, led_timer_lm, start_timer, \
end_timer, onSessionTimer, oldOnSessionTimer; //timer needed for IR leds blinkin

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
uint8_t brightness = 0; // how bright the LED is
int8_t fade_amount = 5; // how many points to fade the LED by
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
char ssid[] = SECRET_SSID; // your network SSID (name)
char pass[] = SECRET_PASS; // your network password (use for WPA, or use as key for WEP)

byte mac[6]; // the MAC address of your Wifi shield

int led =  LED_BUILTIN;
bool getDataFlag = false;

String currentLine = "";

int status = WL_IDLE_STATUS;

Switch bttn = Switch(BUTTON_PIN);

bool isLEDOn = false;
bool calibrationFlag = false;
bool powerSaveMode = false;

unsigned long ledLongLow = 0;
unsigned long tryConnectionTimer = 0;

bool sessionMode = false;
bool waitMode = true;
int8_t connectionAttempts = 3;

void setup()
{
  //Watchdog.enable(WATCHDOG_SETUP_TIMEOUT);
  if (SERIAL_VERBOSE_MODE)
  {
    Serial.begin(115200);
  }

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

  for (int8_t i = 0; i < IMUS_NUMBER; i++)
  {
    digitalWrite(sa0[i], HIGH);
    checkIfIMUConnected(i);
    digitalWrite(sa0[i], LOW);
  }

  for (int8_t i = 0; i < 6; i++)
  {
    grPrint(connected_imu_ids[i]);
  }

  oldOnSessionTimer = 0;

  //battery setup
  currentBatteryLevel = getBattLevel();

  // set up and start timer interrupt
//  startTimer(50);

  //Watchdog.disable();
  //Watchdog.enable(WATCHDOG_LOOP_TIMEOUT);
}

void loop()
{
  //getData();
  // reset watchdog so it knows controller is not hanging
  pollAll(); // polling buttons has priority
  buttonActions();
  handleBtn();

  //Watchdog.reset();
  if (millis() - battTimer >= battMeasurePeriod)
  {
    battTimer = millis();
    // updatePowerMode();
    ledBlink();
  }

  // grPrint("---------------------------------------CLIENT WAIT");
  if (sessionMode)
  {
    pollAll(); // polling buttons has priority
    buttonActions();
    handleBtn();
    if (status != WL_CONNECTED)
    {
      // Disable watchdog while trying to connect since it takes more time
      //Watchdog.disable();
      tryToConnect();
      grPrint("\nStarting connection to server...");
      // and enable watchdog it again
      //Watchdog.enable(WATCHDOG_LOOP_TIMEOUT);
    }
    else
    {
      pollAll(); // polling buttons has priority
      buttonActions();
      handleBtn();
      ledBlink();

      //WiFiClient
      client = server.available();   // listen for incoming clients
      // grPrint("---------------------------------------CLIENT SESSION");
      if (client)
      {
        // if you get a client,
        String currentLine = ""; // make a String to hold incoming data from the client

        while (client.connected()) // loop while the client's connected
        {
          pollAll(); // polling buttons has priority
          buttonActions();
          handleBtn();
          unsigned long time = millis();
          grPrint(time - oldTime);
          oldTime = time;

          // Reset in while loop since we might spend some time in here
          //Watchdog.reset();

          // grPrint("------------------------------------------CLIENT CONNECTED");

          // Handle button events while connected
          handleBtn();

          ledBlink();

          if (millis() - battTimer >= battMeasurePeriod)
          {
            battTimer = millis();
            currentBatteryLevel = getBattLevel();
          }

          checkIncomingEvent(&client);

          if (waitMode || powerSaveMode) // check if in wait or low power mode and skip sending data
          {
            continue;
          }

          onSessionTimer = millis();
          if (getDataFlag && onSessionTimer - oldOnSessionTimer >= 25)
          {
            // grPrint(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>____");
            oldOnSessionTimer = onSessionTimer;
            getData();
            resetSa0();

            generatePackage();
            // client.println(output_data);
            client.write(output_data, sizeof(output_data));
          }
        }

        // close the connection:
        getDataFlag = false;
        client.stop();
        grPrint("client disconnected");
      }
    }
  }
  else if (waitMode)
  {
    ledBlink();
  }
}
