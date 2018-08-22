typedef struct IMU
{
  unsigned long time_stamp;
  short A_AN[3];
  short G_AN[3];
  short G_AN_OFFSET[3]={0, 0, 0};
  short gyro_x = 0;
  short gyro_y = 0;
  short gyro_z = 0;
  short acc_x = 0;
  short acc_y = 0;
  short acc_z = 0;
  short mag_x = 0;
  short mag_y = 0;
  short mag_z = 0;
};
