struct sensor {
  int counter=0;
  long timer=0;
  long timer_old;

  int AN[6];
  int AN_OFFSET[6]={0, 0, 0, 0, 0, 0};

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
