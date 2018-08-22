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

void generatePackage(byte imu_id)
{
  memset(output_data, 0, sizeof(output_data));

  snprintf(output_data, sizeof(output_data), "%d%d%d%d%d%d%d%d%d",
           imu_id,
           highByte(IMUS[imu_id].gyro_x), lowByte(IMUS[imu_id].gyro_x),
           highByte(IMUS[imu_id].gyro_y), lowByte(IMUS[imu_id].gyro_y),
           highByte(IMUS[imu_id].gyro_z), lowByte(IMUS[imu_id].gyro_z),
           highByte(IMUS[imu_id].acc_x), lowByte(IMUS[imu_id].acc_x),
           highByte(IMUS[imu_id].acc_y), lowByte(IMUS[imu_id].acc_y),
           highByte(IMUS[imu_id].acc_z), lowByte(IMUS[imu_id].acc_z));
}



