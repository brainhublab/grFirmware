#include <LSM6.h>
#include <LIS3MDL.h>

LSM6 gyro_acc;
LIS3MDL mag;


void I2C_Init() {
  Wire.begin();
}

void TCA_Select(uint8_t port) {
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << port);
  Wire.endTransmission();
}

void Gyro_Init() {
  // Accel_Init() should have already called gyro_acc.init() and enableDefault()
  gyro_acc.writeReg(LSM6::CTRL2_G, 0x4C); // 104 Hz, 2000 dps full scale
}

void Read_Gyro(uint8_t i) {
  gyro_acc.readGyro();

  SENSORS[i].AN[0] = gyro_acc.g.x;
  SENSORS[i].AN[1] = gyro_acc.g.y;
  SENSORS[i].AN[2] = gyro_acc.g.z;

  SENSORS[i].gyro_x = SENSOR_SIGN[0] * (SENSORS[i].AN[0] - SENSORS[i].AN_OFFSET[0]);
  SENSORS[i].gyro_y = SENSOR_SIGN[1] * (SENSORS[i].AN[1] - SENSORS[i].AN_OFFSET[1]);
  SENSORS[i].gyro_z = SENSOR_SIGN[2] * (SENSORS[i].AN[2] - SENSORS[i].AN_OFFSET[2]);
}

void Accel_Init() {
  gyro_acc.init();
  gyro_acc.enableDefault();
  gyro_acc.writeReg(LSM6::CTRL1_XL, 0x3C); // 52 Hz, 8 g full scale
}

// Reads x,y and z accelerometer registers
void Read_Accel(uint8_t i) {
  gyro_acc.readAcc();

  SENSORS[i].AN[3] = gyro_acc.a.x >> 4; // shift left 4 bits to use 12-bit representation (1 g = 256)
  SENSORS[i].AN[4] = gyro_acc.a.y >> 4;
  SENSORS[i].AN[5] = gyro_acc.a.z >> 4;

  SENSORS[i].accel_x = SENSOR_SIGN[3] * (SENSORS[i].AN[3] - SENSORS[i].AN_OFFSET[3]);
  SENSORS[i].accel_y = SENSOR_SIGN[4] * (SENSORS[i].AN[4] - SENSORS[i].AN_OFFSET[4]);
  SENSORS[i].accel_z = SENSOR_SIGN[5] * (SENSORS[i].AN[5] - SENSORS[i].AN_OFFSET[5]);
}

void Compass_Init() {
  mag.init();
  mag.enableDefault();
}

void Read_Compass(uint8_t i) {
  mag.read();

  SENSORS[i].magnetom_x = SENSOR_SIGN[6] * mag.m.x;
  SENSORS[i].magnetom_y = SENSOR_SIGN[7] * mag.m.y;
  SENSORS[i].magnetom_z = SENSOR_SIGN[8] * mag.m.z;
}

