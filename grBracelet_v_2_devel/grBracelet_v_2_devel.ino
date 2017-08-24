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

// переменные для калмана
float varVolt_x = 1.51455586395835;// 50.23;  // среднее отклонение (ищем в excel)
float varProcess_x = 0.5; // скорость реакции на изменение (подбирается вручную)process noise
float Pc_x = 0.0;
float G_x = 0.0;
float P_x = 1.0;
float Xp_x = 0.0;
float Zp_x = 0.0;
float Xe_x = 0.0;
//
float varVolt_y = 0.721797032292015 ;// 50.23;  // среднее отклонение (ищем в excel)
float varProcess_y = 0.5; // скорость реакции на изменение (подбирается вручную)process noise
float Pc_y = 0.0;
float G_y = 0.0;
float P_y = 1.0;
float Xp_y = 0.0;
float Zp_y = 0.0;
float Xe_y = 0.0;
//
float varVolt_z = 2.6898082531846;  // среднее отклонение (ищем в excel)
float varProcess_z = 0.5; // скорость реакции на изменение (подбирается вручную)process noise
float Pc_z = 0.0;
float G_z = 0.0;
float P_z = 1.0;
float Xp_z = 0.0;
float Zp_z = 0.0;
float Xe_z = 0.0;

float filter_x(float val_x) {  //функция фильтрации
  Pc_x = P_x + varProcess_x;
  G_x = Pc_x/(Pc_x + varVolt_x);
  P_x = (1-G_x)*Pc_x;
  Xp_x = Xe_x;
  Zp_x = Xp_x;
  Xe_x = G_x*(val_x-Zp_x)+Xp_x; // "фильтрованное" значение
  return(Xe_x);
}

float filter_y(float val_y) {  //функция фильтрации
  Pc_y = P_y + varProcess_y;
  G_y = Pc_y/(Pc_y + varVolt_y);
  P_y = (1-G_y)*Pc_y;
  Xp_y = Xe_y;
  Zp_y = Xp_y;
  Xe_y = G_y*(val_y-Zp_y)+Xp_y; // "фильтрованное" значение
  return(Xe_y);
}

float filter_z(float val_z) {  //функция фильтрации
  Pc_z = P_z + varProcess_z;
  G_z = Pc_z/(Pc_z + varVolt_z);
  P_z = (1-G_z)*Pc_z;
  Xp_z = Xe_z;
  Zp_z = Xp_z;
  Xe_z = G_z*(val_z-Zp_z)+Xp_z; // "фильтрованное" значение
  return(Xe_z);
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
