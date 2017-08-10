struct sensor {
  unsigned long timestamp;
  int A_AN[6];
  int A_AN_OFFSET[6]={0, 0, 0, 0, 0, 0};
  int G_AN[6];
  int G_AN_OFFSET[6]={0, 0, 0, 0, 0, 0};
  int gyro_x;
  int gyro_y;
  int gyro_z;
  int accel_x;
  int accel_y;
  int accel_z;
  int magnetom_x;
  int magnetom_y;
  int magnetom_z;
};
