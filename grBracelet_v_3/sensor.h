struct sensor {
  unsigned long time_stamp;
  int A_AN[3];
  int G_AN[3];
  int G_AN_OFFSET[3]={0, 0, 0};
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
