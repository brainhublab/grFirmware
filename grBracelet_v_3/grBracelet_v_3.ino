#include "sensor.h"
#include <Wire.h>
#include <SoftwareSerial.h>

extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}


#define STATUS_LED 13
#define IR_LEDS 5
#define LEDS_ON 6
#define SENSORS_N 6
#define SENSORS_INIT_PORT 0
#define TCAADDR 0x70
#define TIMER 50

unsigned long current_time, last_time;

bool finger_is_connected = false;
bool connected_fingers_id[SENSORS_N] = {0, 0, 0, 0, 0, 0};

const int SENSOR_SIGN[9] = {1, -1, -1, 1, -1, -1, 1, -1, -1};//working
//int SENSOR_SIGN[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};

const int FINGER_SENSOR_SIGN[9] = { -1, 1, 1, 1, -1, -1, -1, 1, -1};
sensor SENSORS[SENSORS_N];

SoftwareSerial bSerial(4, 3); //old prototype 9 10
void setup()
{
  bSerial.begin(115200);
  Serial.begin(115200);

  Serial.println("Begin...");

  pinMode(STATUS_LED, OUTPUT);
  pinMode(IR_LEDS, OUTPUT);
  pinMode(LEDS_ON, INPUT);

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

    for (uint8_t addr = 0; addr <= 127; addr++)
    {
      if (addr == TCAADDR) continue;

      //Serial.print(iter); Serial.print("-->"); Serial.print(addr); Serial.print('\n');
      uint8_t data;
      if (! twi_writeTo(addr, &data, 0, 1, 1))
      {

        Serial.print("Found I2C 0x");  Serial.println(addr, HEX); //Serial.println(addr);
        finger_is_connected = true;
      }

    }
    if (finger_is_connected)
    {
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
      connected_fingers_id[i] = 1;
      Serial.print(i);
    }

    finger_is_connected = false;
  }

  delay(200);
  digitalWrite(STATUS_LED, HIGH);
  delay(20);
  
}
void loop()
{

  //current_time = millis();
 // Serial.println(current_time - last_time);
 
  if(digitalRead(LEDS_ON) == HIGH && TIMER <= (current_time - last_time))
  {
    digitalWrite(IR_LEDS, HIGH);
    last_time = current_time;
    }
  for (uint8_t i = 0; i < SENSORS_N; i++)
  {
    if (connected_fingers_id[i])
    {
      TCA_Select(i);

      Read_Gyro(i);
      Read_Accel(i);
      Read_Compass(i);

      SENSORS[i].time_stamp = millis();
      
      printdata(i);
    }
  }
 digitalWrite(IR_LEDS, LOW);
  
}
