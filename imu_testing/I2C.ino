#include <LSM6.h>
#include <LIS3MDL.h>

LSM6 gyro_acc;
LIS3MDL mag;


void I2C_Init() {
  Wire.begin();
}


void Gyro_Init() {
  // Accel_Init() should have already called gyro_acc.init() and enableDefault()
  gyro_acc.writeReg(LSM6::CTRL2_G, 0x4C); // 104 Hz, 2000 dps full scale
}

void Read_Gyro() {
  gyro_acc.readGyro();
  SENSOR.G_AN[0] = gyro_acc.g.x;
  SENSOR.G_AN[1] = gyro_acc.g.y;
  SENSOR.G_AN[2] = gyro_acc.g.z;

    SENSOR.gyro_x = SENSOR_SIGN[0] * (SENSOR.G_AN[0] - SENSOR.G_AN_OFFSET[0]);
    SENSOR.gyro_y = SENSOR_SIGN[1] * (SENSOR.G_AN[1] - SENSOR.G_AN_OFFSET[1]);
    SENSOR.gyro_z = SENSOR_SIGN[2] * (SENSOR.G_AN[2] - SENSOR.G_AN_OFFSET[2]);

 }

void Accel_Init() {
  gyro_acc.init();
  gyro_acc.enableDefault();
  gyro_acc.writeReg(LSM6::CTRL1_XL, 0x3C); // 52 Hz, 8 g full scale A0 for 2G
}

void Read_Accel( ) {
  gyro_acc.readAcc();
  SENSOR.A_AN[3] = gyro_acc.a.x;
  SENSOR.A_AN[4] = gyro_acc.a.y;
  SENSOR.A_AN[5] = gyro_acc.a.z;
  
    SENSOR.accel_x = SENSOR_SIGN[3] * (SENSOR.A_AN[3] - SENSOR.A_AN_OFFSET[3]);
    SENSOR.accel_y = SENSOR_SIGN[4] * (SENSOR.A_AN[4] - SENSOR.A_AN_OFFSET[4]);
    SENSOR.accel_z = SENSOR_SIGN[5] * (SENSOR.A_AN[5] - SENSOR.A_AN_OFFSET[5]);
 
}

void Compass_Init() {
  mag.init();
  mag.enableDefault();
}

void Read_Compass() {
  mag.read();


    SENSOR.magnetom_x = SENSOR_SIGN[6] * mag.m.x;
    SENSOR.magnetom_y = SENSOR_SIGN[7] * mag.m.y;
    SENSOR.magnetom_z = SENSOR_SIGN[8] * mag.m.z;
 
}
