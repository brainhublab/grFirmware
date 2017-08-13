struct sensor {
  unsigned long timestamp;
  float A_AN[6];
  float A_AN_OFFSET[6]={0, 0, 0, 0, 0, 0};
  float G_AN[6];
  float G_AN_OFFSET[6]={0, 0, 0, 0, 0, 0};
  float gyro_x;
  float gyro_y;
  float gyro_z;
  float accel_x;
  float accel_y;
  float accel_z;
  float magnetom_x;
  float magnetom_y;
  float magnetom_z;
};
