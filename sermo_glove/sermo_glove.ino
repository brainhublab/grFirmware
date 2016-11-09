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

#include "BluefruitConfig.h"
/*including compas acclerometer and gyro */
#include <Wire.h>
#include <LSM303.h>
#include <L3G.h>
LSM303 compass;//define compas
L3G gyro;

//containers for data report
char report_gyro_and_compass[80]; //size of bufer
char report_fingers[20];
char report_gyro[20];
char report_magnet[20];
char report_accelerometer[20];


/*=========================================================================
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1 //sometimes needs to chage
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

int8_t sensorsServiceId;
uint8_t sensorsServiceUUID[] {0xfc, 0xed, 0x64, 0x08, 0xc0, 0x15, 0x45, 0xea, 0xb5, 0x0d, 0x1d, 0xe3, 0x2a, 0x1c, 0x2f, 0x6d};
int8_t fingersCharId;
uint8_t fingersCharUUID[] {0xfc, 0xed, 0x64, 0x09, 0xc0, 0x15, 0x45, 0xea, 0xb5, 0x0d, 0x1d, 0xe3, 0x2a, 0x1c, 0x2f, 0x6d};
int8_t gyroCharId;
uint8_t gyroCharUUID[] {0xfc, 0xed, 0x64, 0x0a, 0xc0, 0x15, 0x45, 0xea, 0xb5, 0x0d, 0x1d, 0xe3, 0x2a, 0x1c, 0x2f, 0x6d};
int8_t accelerometerCharId;
uint8_t accelerometerCharUUID[] {0xfc, 0xed, 0x64, 0x0b, 0xc0, 0x15, 0x45, 0xea, 0xb5, 0x0d, 0x1d, 0xe3, 0x2a, 0x1c, 0x2f, 0x6d};
int8_t magnetChatId;
uint8_t magnetChatUUID[] {0xfc, 0xed, 0x64, 0x0c, 0xc0, 0x15, 0x45, 0xea, 0xb5, 0x0d, 0x1d, 0xe3, 0x2a, 0x1c, 0x2f, 0x6d};

// GATT service setup
void setup_gatt()
{
  ble.setInterCharWriteDelay(5);

  ble.atcommand("AT+GAPDEVNAME=Gestus[R]");

  Serial.println("Adding sensors service");
  sensorsServiceId = gatt.addService(sensorsServiceUUID);
  if(sensorsServiceId == 0)
  {
    Serial.println(sensorsServiceId);
    ble.atcommand("AT+GATTLIST");
    error(F("Failed to create sensors service "));
  }

  Serial.println("Adding fingers characteristic");
  fingersCharId = gatt.addCharacteristic(fingersCharUUID, GATT_CHARS_PROPERTIES_READ, 1, 20, BLE_DATATYPE_STRING);
  if(fingersCharId == 0)
  {
    error(F("Failed to create fingers characteristic"));
  }

  Serial.println("Adding gyroscope characteristic");
  gyroCharId = gatt.addCharacteristic(gyroCharUUID, GATT_CHARS_PROPERTIES_READ, 1, 20, BLE_DATATYPE_STRING);
  if(gyroCharId == 0)
  {
    error(F("Failed to create gyroscope characteristic"));
  }

  Serial.println("Adding accelerometer characteristic");
  accelerometerCharId = gatt.addCharacteristic(accelerometerCharUUID, GATT_CHARS_PROPERTIES_READ, 1, 20, BLE_DATATYPE_STRING);
  if(accelerometerCharId == 0)
  {
    error(F("Failed to create accelerometer characteristic"));
  }

  Serial.println("Adding magnetometer characteristic");
  magnetChatId = gatt.addCharacteristic(magnetChatUUID, GATT_CHARS_PROPERTIES_READ, 1, 20, BLE_DATATYPE_STRING);
  if(magnetChatId == 0)
  {
    error(F("Failed to create magnetometer characteristic"));
  }

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

  //init compas data
  Wire.begin();
  compass.init();
  compass.enableDefault();

  //init gyro data
  if (!gyro.init())
  {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }

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
  //read and organize compas data
  compass.read();
  gyro.read();
  snprintf(report_gyro_and_compass, sizeof(report_gyro_and_compass), "%d %d %d %d %d %d %d %d %d %d",
    compass.a.x, compass.a.y, compass.a.z,
    compass.m.x, compass.m.y, compass.m.z,
    gyro.g.x, gyro.g.y, gyro.g.z, 5);

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
  Serial.println(report_gyro_and_compass);
  //ble.print(report_gyro_and_compass);//put data in bluetooth stream
  delay(100);

  // Echo received data
  while ( ble.available() )
  {
    int c = ble.read();

    Serial.print((char)c);

    // Hex output too, helps w/debugging!
    Serial.print(" [0x");
    if (c <= 0xF) Serial.print(F("0"));
    Serial.print(c, HEX);
    Serial.print("] ");
  }
}
