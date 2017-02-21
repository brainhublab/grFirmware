#include <Wire.h>
#include <LSM6.h>
#include <LIS3MDL.h>

#define TCAADDR 0x70
#define PORT_1 0
#define PORT_2 5

LIS3MDL compass1, compass2;
LSM6 gyro1, gyro2;

char report_gyro_and_compass[80];

void tca_select(uint8_t i) {
  if(i > 7) return;

  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void print_reports(LIS3MDL *compass, LSM6 *gyro) {
  compass->read();
  gyro->read();
  snprintf(report_gyro_and_compass, sizeof(report_gyro_and_compass), "%d %d %d %d %d %d %d %d %d",
    gyro->a.x, gyro->a.y, gyro->a.z,
    gyro->g.x, gyro->g.y, gyro->g.z,
    compass->m.x, compass->m.y, compass->m.z);
  Serial.println(report_gyro_and_compass);
}

void setup() {
  delay(500);
  
  Serial.begin(9600);
  
  Wire.begin();

  tca_select(PORT_1);

  if(!compass1.init()) {
    Serial.println("Compass1 failed to init");
  }
  compass1.enableDefault();

  if(!gyro1.init()) {
    Serial.println("Gyro1 failed to init");
  }
  gyro1.enableDefault();
/**
  tca_select(PORT_2);

  if(!compass2.init()) {
    Serial.println("Compass2 failed to init");
  }
  compass2.enableDefault();

  if(!gyro2.init()) {
    Serial.println("Gyro2 failed to init");
  }
  gyro2.enableDefault();
**/
  Serial.println("Setup OK!");
}

void loop() {
  delay(100);
  tca_select(PORT_1);
  Serial.println("TCA port #0");
  print_reports(&compass1, &gyro1);
/**
  tca_select(PORT_2);
  Serial.println("TCA port #1");
  print_reports(&compass2, &gyro2);
**/
}
