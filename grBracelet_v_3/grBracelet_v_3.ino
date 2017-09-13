#include "sensor.h"
#include <Wire.h>
#include <SoftwareSerial.h>

#define STATUS_LED 13
#define SENSORS_N 6
#define SENSORS_INIT_PORT 0
#define TCAADDR 0x70

const int SENSOR_SIGN[9] = {1, -1, -1, 1, -1, -1, 1, -1, -1};
//int SENSOR_SIGN[9] = {1, 1, -1, 1, 1, -1, 1, 1, 1};

const int FINGER_SENSOR_SIGN[9] = { -1, 1, 1, 1, -1, -1, -1, 1, -1};
sensor SENSORS[SENSORS_N];

SoftwareSerial bSerial(4, 3); //old prototype 9 10
void setup()
{
  bSerial.begin(115200);
  Serial.begin(115200);

  Serial.println("Begin...");

  pinMode (STATUS_LED, OUTPUT);

  I2C_Init();

  digitalWrite(STATUS_LED, LOW);
  delay(200);

  for (uint8_t i = 0; i < SENSORS_N; i++)
  {
    TCA_Select(i);

    Accel_Init();
    Compass_Init();
    Gyro_Init();

    digitalWrite(STATUS_LED, HIGH);
    delay(200);
    digitalWrite(STATUS_LED, LOW);

    for (uint8_t j = 0; j < 32; j++)
    {
      Read_Gyro(i);
      for (uint8_t y = 0; y < 3; y++)
      {
        SENSORS[i].G_AN_OFFSET[y] += SENSORS[i].G_AN[y];
      }
      delay(20);
    }
    for (uint8_t y = 0; y < 3; y++)
    {
      SENSORS[i].G_AN_OFFSET[y] = SENSORS[i].G_AN_OFFSET[y] / 32;
    }
      Serial.print(i);
  }

  delay(200);
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
