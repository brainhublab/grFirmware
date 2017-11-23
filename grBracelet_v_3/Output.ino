/*printing data via software serial port to bluetooth module*/
void printdata(uint8_t imu_index)
{
  if(imu_index==PALM_INDEX)
  {
    Serial.print(imu_index);
    Serial.print(" ");
    Serial.print(IMUS[imu_index].gyro_x);
    Serial.print(" ");
    Serial.print(IMUS[imu_index].gyro_y);
    Serial.print(" ");
    Serial.print(IMUS[imu_index].gyro_z);
    Serial.print(" ");
    Serial.print(IMUS[imu_index].accel_x);
    Serial.print (" ");
    Serial.print(IMUS[imu_index].accel_y);
    Serial.print (" ");
    Serial.print(IMUS[imu_index].accel_z);
    Serial.print(" ");
    Serial.print(IMUS[imu_index].magnetom_x);
    Serial.print (" ");
    Serial.print(IMUS[imu_index].magnetom_y);
    Serial.print (" ");
    Serial.print(IMUS[imu_index].magnetom_z);
    Serial.print(" ");
    Serial.print(IMUS[imu_index].time_stamp);
    Serial.print('\n');
  }
  else
  {
    Serial.print(imu_index);
    Serial.print(" ");
    Serial.print(IMUS[imu_index].gyro_z*0); // y
    Serial.print(" ");
    Serial.print(IMUS[imu_index].gyro_x*0); // z
    Serial.print(" ");
    Serial.print(IMUS[imu_index].gyro_y);
    Serial.print(" ");
    Serial.print(IMUS[imu_index].accel_z*0);
    Serial.print (" ");
    Serial.print(IMUS[imu_index].accel_x*0);
    Serial.print (" ");
    Serial.print(IMUS[imu_index].accel_y);
    Serial.print(" ");
    Serial.print(IMUS[imu_index].magnetom_z*0);
    Serial.print (" ");
    Serial.print(IMUS[imu_index].magnetom_x*0);
    Serial.print (" ");
    Serial.print(IMUS[imu_index].magnetom_y);
    Serial.print(" ");
    Serial.print(IMUS[imu_index].time_stamp);
    Serial.print('\n');
  }
}
