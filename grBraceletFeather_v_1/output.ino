void grPrint(String output)
{
  if (SERIAL_VERBOSE_MODE)
  {
    Serial.println(output);
  }
}

// A small helper
void error(const __FlashStringHelper*err)
{
  Serial.println(err);
  while (1);
}

void generatePackage(int8_t imu_id)
{
 /*
  IMUS[imu_id].gyro_x = 12345;
  IMUS[imu_id].gyro_y = 12345;
  IMUS[imu_id].gyro_z = 12345;

  IMUS[imu_id].acc_x = 12345;
  IMUS[imu_id].acc_y = 12345;
  IMUS[imu_id].acc_z = 12345;

  IMUS[imu_id].mag_x = 12345;
  IMUS[imu_id].mag_y = 12345;
  IMUS[imu_id].mag_z = 12345;
 */
  memset(output_data, 0, sizeof(output_data));

  output_data[0] = (uint8_t)imu_id;
  output_data[1] = (uint8_t)((IMUS[imu_id].gyro_x >> 8) & 0xff);
  output_data[2] = (uint8_t)((IMUS[imu_id].gyro_x) & 0xff);
  output_data[3] = (uint8_t)((IMUS[imu_id].gyro_y >> 8) & 0xff);
  output_data[4] = (uint8_t)((IMUS[imu_id].gyro_y) & 0xff);
  output_data[5] = (uint8_t)((IMUS[imu_id].gyro_z >> 8) & 0xff);
  output_data[6] = (uint8_t)((IMUS[imu_id].gyro_z) & 0xff);

  output_data[7] = (uint8_t)((IMUS[imu_id].acc_x >> 8) & 0xff);
  output_data[8] = (uint8_t)((IMUS[imu_id].acc_x) & 0xff);
  output_data[9] = (uint8_t)((IMUS[imu_id].acc_y >> 8) & 0xff);
  output_data[10] = (uint8_t)((IMUS[imu_id].acc_y) & 0xff);
  output_data[11] = (uint8_t)((IMUS[imu_id].acc_z >> 8) & 0xff);
  output_data[12] = (uint8_t)((IMUS[imu_id].acc_z) & 0xff);
  

  output_data[13] = (uint8_t)((IMUS[imu_id].mag_x >> 8) & 0xff);
  output_data[14] = (uint8_t)((IMUS[imu_id].mag_x) & 0xff);
  output_data[15] = (uint8_t)((IMUS[imu_id].mag_y >> 8) & 0xff);
  output_data[16] = (uint8_t)((IMUS[imu_id].mag_y) & 0xff);
  output_data[17] = (uint8_t)((IMUS[imu_id].mag_z >> 8) & 0xff);
  output_data[18] = (uint8_t)((IMUS[imu_id].mag_z) & 0xff);

 
  //   output_data[0] =
/*
  int16_t result = ((output_data[1] & 0xFF) << 8 | (output_data[2] & 0xFF)  ) ;
  Serial.println("|------------------|");

  Serial.println(result);
  Serial.println("------------------");
  Serial.println(IMUS[imu_id].gyro_x);
  Serial.println("|------------------|");
*/

  // "%d%d%d%d%d%d%d%d%d%d%d%d%d"   "%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d"
  //"%c%c%c%c%c%c%c%c%c%c%c%c%c"    "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"

}
