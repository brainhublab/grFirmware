#include "sensor.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#define STATUS_LED 13
#define chargePin 5
#define SENSORS_N 1

int SENSOR_SIGN[9] = {1, 1, -1, -1, -1, 1, 1, 1, 1};
sensor SENSOR;
SoftwareSerial bSerial(4, 3);

// переменные для калмана
float varVolt = 50.2368536610182;  // среднее отклонение (ищем в excel)
float varProcess = 1.0; // скорость реакции на изменение (подбирается вручную)process noise
float Pc = 0.0;
float G = 0.0;
float P = 1.0;
float Xp = 0.0;
float Zp = 0.0;
float Xe = 0.0;

float filter(float val) {  //функция фильтрации
  Pc = P + varProcess;
  G = Pc/(Pc + varVolt);
  P = (1-G)*Pc;
  Xp = Xe;
  Zp = Xp;
  Xe = G*(val-Zp)+Xp; // "фильтрованное" значение
  return(Xe);
}
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

    Accel_Init();
    Compass_Init();
    Gyro_Init();
    for (int l = 0; l < 10; l++)
    {
      Read_Gyro();
      Read_Accel();
    }
    digitalWrite(STATUS_LED, HIGH);
    delay(500);
    digitalWrite(STATUS_LED, LOW);
    for (uint8_t j = 0; j < 32; j++)
    {
      Read_Gyro();
      Read_Accel();
      for (uint8_t y = 0; y < 6; y++)
      {
        SENSOR.A_AN_OFFSET[y] += SENSOR.A_AN[y];
        SENSOR.G_AN_OFFSET[y] += SENSOR.G_AN[y];
      }
      delay(40);
    }
    for (uint8_t y = 0; y < 6; y++)
    {
      SENSOR.A_AN_OFFSET[y] = SENSOR.A_AN_OFFSET[y] / 32;
      SENSOR.G_AN_OFFSET[y] = SENSOR.G_AN_OFFSET[y] / 32;
    }
  
  
  delay(2000);
  digitalWrite(STATUS_LED, HIGH);
  delay(20);



}
void loop()
{
   
 
    Read_Gyro();
    Read_Accel();
    Read_Compass();
    SENSOR.timestamp = millis();
    printdata();
 
}
