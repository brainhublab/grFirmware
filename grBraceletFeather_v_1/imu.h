typedef struct IMU
{
  Madgwick m_filter;
  unsigned long time_stamp;
  int16_t A_AN[3];
  int16_t G_AN[3];
  int16_t G_AN_OFFSET[3] = {0, 0, 0};
  int16_t gyro_x = 0;
  int16_t gyro_y = 0;
  int16_t gyro_z = 0;
  int16_t acc_x = 0;
  int16_t acc_y = 0;
  int16_t acc_z = 0;
  int16_t mag_x = 0;
  int16_t mag_y = 0;
  int16_t mag_z = 0;
};
