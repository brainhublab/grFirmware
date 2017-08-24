#include "sensor.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#define STATUS_LED 13
#define chargePin 5
#define SENSORS_N 6
#define SENSORS_INIT_PORT 0
#define TCAADDR 0x70

//int SENSOR_SIGN[9] = {1, 1, -1, -1, -1, 1, 1, 1, 1};
int SENSOR_SIGN[9] = {1, 1, -1, 1, 1, -1, 1, 1, 1};

int FINGER_SENSOR_SIGN[9] = { -1, 1, 1, 1, -1, -1, -1, 1, -1};
sensor SENSORS[SENSORS_N];
SoftwareSerial bSerial(9, 10);
void setup()
{
  bSerial.begin(115200);
  Serial.begin(115200);
  pinMode (STATUS_LED, OUTPUT);
  pinMode(chargePin, OUTPUT);
  digitalWrite(chargePin, HIGH);
  I2C_Init();

  digitalWrite(STATUS_LED, LOW);
  delay(2000);
  for (uint8_t i = 0; i < SENSORS_N; i++)
  {
    TCA_Select(i);
    Accel_Init();
    Compass_Init();
    Gyro_Init();
    for (int l = 0; l < 10; l++)
    {
      Read_Gyro(i);
      Read_Accel(i);
    }
    digitalWrite(STATUS_LED, HIGH);
    delay(500);
    digitalWrite(STATUS_LED, LOW);
    for (uint8_t j = 0; j < 32; j++)
    {
      Read_Gyro(i);
      Read_Accel(i);
      for (uint8_t y = 0; y < 6; y++)
      {
        SENSORS[i].A_AN_OFFSET[y] += SENSORS[i].A_AN[y];
        SENSORS[i].G_AN_OFFSET[y] += SENSORS[i].G_AN[y];
      }
      delay(40);
    }
    for (uint8_t y = 0; y < 6; y++)
    {
      SENSORS[i].A_AN_OFFSET[y] = SENSORS[i].A_AN_OFFSET[y] / 32;
      SENSORS[i].G_AN_OFFSET[y] = SENSORS[i].G_AN_OFFSET[y] / 32;
    }
  }
  
  delay(2000);
  digitalWrite(STATUS_LED, HIGH);
  delay(20);



}
void loop()
{
   
  for (uint8_t i = 0; i < SENSORS_N; i++)
  {
    TCA_Select(i);
    Read_Gyro(i);
    Read_Accel(i);
    Read_Compass(i);
    SENSORS[i].timestamp = millis();
    printdata(i);
  }
}
