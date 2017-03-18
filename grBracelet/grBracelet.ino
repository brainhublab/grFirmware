/*include arduino libs*/
#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

/*including bluetooth libs and config header*/
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEGatt.h"
#include "bluefruitConfig.h" // config header

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
int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1};

//containers for data report TODO  move t to struct
char report_gyro[20];
char report_magnet[20];
char report_accelerometer[20];

//Some defines for gyroscope data
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

#define TCAADDR 0x70

// LSM303/LIS3MDL magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 or LIS3MDL library to find the right values for your board

#define M_X_MIN -1000
#define M_Y_MIN -1000
#define M_Z_MIN -1000
#define M_X_MAX +1000
#define M_Y_MAX +1000
#define M_Z_MAX +1000

/*=========================================================================
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0 //sometimes needs to chage
    // #define GAPDEVNAME "[R]_SERMO_"
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines

SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

Adafruit_BLEGatt gatt(ble);

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


int8_t sensorsServiceId;
uint8_t sensorsServiceUUID[] {0xfc, 0xed, 0x64, 0x08, 0xc0, 0x15, 0x45, 0xea, 0xb5, 0x0d, 0x1d, 0xe3, 0x2a, 0x1c, 0x2f, 0x6d};

sensor allSensors[6];

void generate_sensors_uuids(uint8_t serviceUUID[], sensor arr[]) 
{
  Serial.println("Generating UUIDs");
  for(int i=0; i < 6; i++) 
  {
    sensor s;

    memcpy(s.gyroUUID, serviceUUID, 16);
    s.gyroUUID[3] += i * 0x03 + 0x01;
    memcpy(s.accUUID, serviceUUID, 16);
    s.accUUID[3] += i * 3 + 2;
    memcpy(s.magUUID, serviceUUID, 16);
    s.magUUID[3] += i * 3 + 3;

    arr[i] = s;
  }
}

void init_characteristics(sensor arr[]) 
{
  for(int i = 0; i < 6; i++) 
  {
    Serial.print("Sensor "); Serial.println(i);
    
    Serial.println("- adding gyro characteristic");
    arr[i].gyroId = gatt.addCharacteristic(arr[i].gyroUUID, GATT_CHARS_PROPERTIES_READ, 1, 20, BLE_DATATYPE_STRING);
    if(arr[i].gyroId == 0)
    {
      error(F("Failed"));
    }
    Serial.println("- adding accelerometer characteristic");
    arr[i].accId = gatt.addCharacteristic(arr[i].accUUID, GATT_CHARS_PROPERTIES_READ, 1, 20, BLE_DATATYPE_STRING);
    if(arr[i].accId == 0)
    {
      error(F("Failed"));
    }
    Serial.println("- adding magnetometer characteristic");
    arr[i].magId = gatt.addCharacteristic(arr[i].magUUID, GATT_CHARS_PROPERTIES_READ, 1, 20, BLE_DATATYPE_STRING);
    if(arr[i].magId == 0)
    {
      error(F("Failed"));
    }
  }
}

void read_finger(uint8_t port, sensor *s) {
  tca_select(port);

  compass.read();
  gyro.read();
  snprintf(report_gyro, sizeof(report_gyro), "%d %d %d",
    gyro.g.x, gyro.g.y, gyro.g.z);
  snprintf(report_accelerometer, sizeof(report_accelerometer), "%d %d %d",
    gyro.a.x, gyro.a.y, gyro.a.z);
  snprintf(report_magnet, sizeof(report_magnet), "%d %d %d",
    compass.m.x, compass.m.y, compass.m.z);

  Serial.println(report_gyro);

  gatt.setChar(s->gyroId, report_gyro);
  gatt.setChar(s->accId, report_accelerometer);
  gatt.setChar(s->magId, report_magnet);

}

// GATT service setup
void setup_gatt()
{
  ble.setInterCharWriteDelay(5);

  ble.atcommand("AT+GAPDEVNAME=Gestus[R]");
  ble.atcommand("AT+GATTCLEAR");

  generate_sensors_uuids(sensorsServiceUUID, allSensors);

  Serial.println("Adding sensors service");
  sensorsServiceId = gatt.addService(sensorsServiceUUID);
  if(sensorsServiceId == 0)
  {
    Serial.println(sensorsServiceId);
    ble.atcommand("AT+GATTLIST");
    error(F("Failed to create sensors service "));
  }
  ble.atcommand("AT+GATTLIST");

  init_characteristics(allSensors);

  // Serial.println("Adding sensors service UUID to the advertising payload");
  // uint8_t advdata[] { 0x02, 0x01, 0x06, 0x11, 0x06, 0x6d, 0x2f, 0x1c, 0x2a, 0xe3, 0x1d, 0x0d, 0xb5, 0xea, 0x45, 0x15, 0xc0, 0x08, 0x64, 0xfc };
  // ble.setAdvData(advdata, sizeof(advdata));

  Serial.println("Performing SW reset (service changes require reset)");
  ble.reset();
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);
  Serial.begin(115200);

  randomSeed(micros());

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  //ble.verbose(false);  // debug info is a little annoying after this point!

  if ( GATT_RUN_SETUP )
  {
    setup_gatt();
  }

  //init multiplexer
  tca_select(0);


  //init compas data
  Wire.begin();

  
  /*if (!compass.init())
  {
    Serial.println("Failed to autodetect compass type!");
    while (1);
  }*/
  compass.init();
  compass.enableDefault();

  //init gyro data
  /*if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }*/
  gyro.init();
  gyro.enableDefault();

  Serial.println(F("Adafruit Bluefruit Command <-> Data Mode Example"));
  Serial.println(F("------------------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/

void loop(void)
{
  /*
  //read and organize compas data
  compass.read();
  gyro.read();

  
  x = ToRad(gyro.g.x);
  y = ToRad(gyro.g.y);
  z = ToRad(gyro.g.z);
  dtostrf(x, 6, 3, strx);
  dtostrf(y, 6, 3, stry);
  dtostrf(z, 6, 3, strz);

  snprintf(report_gyro, sizeof(report_gyro), "%d %d %d",
    gyro.g.x, gyro.g.y, gyro.g.z);
  snprintf(report_accelerometer, sizeof(report_accelerometer), "%d %d %d",
    compass.a.x, compass.a.y, compass.a.z);
  snprintf(report_magnet, sizeof(report_magnet), "%d %d %d",
    compass.m.x, compass.m.y, compass.m.z);

  gatt.setChar(gyroCharId, report_gyro);
  gatt.setChar(accelerometerCharId, report_accelerometer);
  gatt.setChar(magnetChatId, report_magnet);

   //A, M, gyro
  //ble.print(report_gyro_and_compass);//put data in bluetooth stream
  //delay(100);

  // Echo received data
  */
  for(uint8_t i = 0; i < 6; i++) {
    read_finger(i, &allSensors[i]);  
  }
}
