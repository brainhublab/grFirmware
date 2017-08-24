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
  SENSORS[i].G_AN[0] = gyro_acc.g.x;
  SENSORS[i].G_AN[1] = gyro_acc.g.y;
  SENSORS[i].G_AN[2] = gyro_acc.g.z;
  if (i == 5)
  {
    SENSORS[i].gyro_x = SENSOR_SIGN[0] * (SENSORS[i].G_AN[0] - SENSORS[i].G_AN_OFFSET[0]);
    SENSORS[i].gyro_y = SENSOR_SIGN[1] * (SENSORS[i].G_AN[1] - SENSORS[i].G_AN_OFFSET[1]);
    SENSORS[i].gyro_z = SENSOR_SIGN[2] * (SENSORS[i].G_AN[2] - SENSORS[i].G_AN_OFFSET[2]);
  }
  else
  {
    SENSORS[i].gyro_x = FINGER_SENSOR_SIGN[0] * (SENSORS[i].G_AN[0] - SENSORS[i].G_AN_OFFSET[0]);
    SENSORS[i].gyro_y = FINGER_SENSOR_SIGN[1] * (SENSORS[i].G_AN[1] - SENSORS[i].G_AN_OFFSET[1]);
    SENSORS[i].gyro_z = FINGER_SENSOR_SIGN[2] * (SENSORS[i].G_AN[2] - SENSORS[i].G_AN_OFFSET[2]);
  }
}

void Accel_Init() {
  gyro_acc.init();
  gyro_acc.enableDefault();
  gyro_acc.writeReg(LSM6::CTRL1_XL, 0x3C); // 3C 52 Hz, 8 g full scale A0 for 2G
}

void Read_Accel(uint8_t i) {
  gyro_acc.readAcc();
  SENSORS[i].A_AN[3] = gyro_acc.a.x >> 4;
  SENSORS[i].A_AN[4] = gyro_acc.a.y >> 4;
  SENSORS[i].A_AN[5] = gyro_acc.a.z >> 4;
  if (i == 5)
  {
    SENSORS[i].accel_x = SENSOR_SIGN[3] * (SENSORS[i].A_AN[3]);// - SENSORS[i].A_AN_OFFSET[3]);
    SENSORS[i].accel_y = SENSOR_SIGN[4] * (SENSORS[i].A_AN[4]);// - SENSORS[i].A_AN_OFFSET[4]);
    SENSORS[i].accel_z = SENSOR_SIGN[5] * (SENSORS[i].A_AN[5]);// - SENSORS[i].A_AN_OFFSET[5]);
  }
  else
  {
    SENSORS[i].accel_x = FINGER_SENSOR_SIGN[3] * (SENSORS[i].A_AN[3] - SENSORS[i].A_AN_OFFSET[3]);
    SENSORS[i].accel_y = FINGER_SENSOR_SIGN[4] * (SENSORS[i].A_AN[4] - SENSORS[i].A_AN_OFFSET[4]);
    SENSORS[i].accel_z = FINGER_SENSOR_SIGN[5] * (SENSORS[i].A_AN[5] - SENSORS[i].A_AN_OFFSET[5]);
  }
}

void Compass_Init() {
  mag.init();
  mag.enableDefault();
}

void Read_Compass(uint8_t i) {
  mag.read();

  if (i == 5)
  {
    SENSORS[i].magnetom_x = SENSOR_SIGN[6] * mag.m.x;
    SENSORS[i].magnetom_y = SENSOR_SIGN[7] * mag.m.y;
    SENSORS[i].magnetom_z = SENSOR_SIGN[8] * mag.m.z;
  }
  else
  {
    SENSORS[i].magnetom_x = FINGER_SENSOR_SIGN[6] * mag.m.x;
    SENSORS[i].magnetom_y = FINGER_SENSOR_SIGN[7] * mag.m.y;
    SENSORS[i].magnetom_z = FINGER_SENSOR_SIGN[8] * mag.m.z;
  }
}
