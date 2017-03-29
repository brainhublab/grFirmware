struct sensor {
  int counter=0;
  long timer=0;
  long timer_old;

  int AN[6];
  int AN_OFFSET[6]={0, 0, 0, 0, 0, 0};

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
