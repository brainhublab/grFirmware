#include <LSM6.h> //pololu accelerometer and gyro lib
#include <LIS3MDL.h> //magnetometer lib

LSM6 gyro_acc; //initialization of accelerometer and gyroscope vars
LIS3MDL mag; //initialization of magnetometer

/*I2C bus */
void I2C_Init()
{
  Wire.begin();
}

/*select port on multiplexer*/
void TCA_Select(uint8_t port) 
{
  Wire.beginTransmission(TCAADDR); //wiring with multiplexer port
  Wire.write(1 << port); //write method from whire.h reads data from selected multiplexer, so we can do bus scanning
  Wire.endTransmission(); //end transmission send data via I2C to controller
}

/*initializing gyroscope*/
void Gyro_Init() 
{
  // Accel_Init() should have already called gyro_acc.init() and enableDefault() for reference LSM6.h
  gyro_acc.writeReg(LSM6::CTRL2_G, 0x3C); // 104 Hz, 2000 dps full scale 4c
}

/*reading data from gyroscope*/
void Read_Gyro(uint8_t imu_index)
{
  gyro_acc.readGyro(); //make reading from IMU for reference LSM6.h

  /*select array from IMUs and read each axis*/
  IMUS[imu_index].G_AN[0] = gyro_acc.g.x;
  IMUS[imu_index].G_AN[1] = gyro_acc.g.y;
  IMUS[imu_index].G_AN[2] = gyro_acc.g.z;

  /*make reading from gyroscope with offset extraction and IMU sign correction of orientation*/
  if (imu_index == PALM_INDEX) //if its == 5 is palm index on multiplexer
  {
    IMUS[imu_index].gyro_x = IMU_SIGN[0] * (IMUS[imu_index].G_AN[0] - IMUS[imu_index].G_AN_OFFSET[0]);
    IMUS[imu_index].gyro_y = IMU_SIGN[1] * (IMUS[imu_index].G_AN[1] - IMUS[imu_index].G_AN_OFFSET[1]);
    IMUS[imu_index].gyro_z = IMU_SIGN[2] * (IMUS[imu_index].G_AN[2] - IMUS[imu_index].G_AN_OFFSET[2]);
  }
  else //it's finger
  {
    IMUS[imu_index].gyro_x = FINGER_IMU_SIGN[0] * (IMUS[imu_index].G_AN[0] - IMUS[imu_index].G_AN_OFFSET[0]);
    IMUS[imu_index].gyro_y = FINGER_IMU_SIGN[1] * (IMUS[imu_index].G_AN[1] - IMUS[imu_index].G_AN_OFFSET[1]);
    IMUS[imu_index].gyro_z = FINGER_IMU_SIGN[2] * (IMUS[imu_index].G_AN[2] - IMUS[imu_index].G_AN_OFFSET[2]);
  }
}

/*initialize accelerometer*/
void Accel_Init() {
  gyro_acc.init(); //initializing
  gyro_acc.enableDefault(); //enable default flags for both type of data gyro and acc
  gyro_acc.writeReg(LSM6::CTRL1_XL, 0x3C); // 3C 52 Hz, 8 g full scale A0 for 2G or 40
}

/* make some readings from accelerometer */
void Read_Accel(uint8_t imu_index) {
  gyro_acc.readAcc();
  /* store readed data from imu with 4 bit shifting */
  IMUS[imu_index].A_AN[0] = gyro_acc.a.x >> 4;
  IMUS[imu_index].A_AN[1] = gyro_acc.a.y >> 4;
  IMUS[imu_index].A_AN[2] = gyro_acc.a.z >> 4;

  /*store data from accelrometer with IMU sign correction of orientation*/
  if (imu_index == PALM_INDEX) //if imu_index == 5 its palm 
  {
    IMUS[imu_index].accel_x = IMU_SIGN[3] * (IMUS[imu_index].A_AN[0]);
    IMUS[imu_index].accel_y = IMU_SIGN[4] * (IMUS[imu_index].A_AN[1]);
    IMUS[imu_index].accel_z = IMU_SIGN[5] * (IMUS[imu_index].A_AN[2]);
  }
  else
  {
    IMUS[imu_index].accel_x = FINGER_IMU_SIGN[3] * (IMUS[imu_index].A_AN[0]);
    IMUS[imu_index].accel_y = FINGER_IMU_SIGN[4] * (IMUS[imu_index].A_AN[1]);
    IMUS[imu_index].accel_z = FINGER_IMU_SIGN[5] * (IMUS[imu_index].A_AN[2]);
  }
}

/*magnetometer initializing*/
void Compass_Init() 
{
  mag.init();
  mag.enableDefault();
}

/*magnetometer reading*/
void Read_Compass(uint8_t imu_index) {
  mag.read();

  if (imu_index == PALM_INDEX)
  {
    IMUS[imu_index].magnetom_x = IMU_SIGN[6] * mag.m.x;
    IMUS[imu_index].magnetom_y = IMU_SIGN[7] * mag.m.y;
    IMUS[imu_index].magnetom_z = IMU_SIGN[8] * mag.m.z;
  }
  else
  {
    IMUS[imu_index].magnetom_x = FINGER_IMU_SIGN[6] * mag.m.x;
    IMUS[imu_index].magnetom_y = FINGER_IMU_SIGN[7] * mag.m.y;
    IMUS[imu_index].magnetom_z = FINGER_IMU_SIGN[8] * mag.m.z;
  }
}
