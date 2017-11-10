/*printing data via software serial port to bluetooth module*/
void printdata(uint8_t imu_index)
{
  if(imu_index==PALM_INDEX)
  {
    bSerial.print(imu_index);
    bSerial.print(" ");
    bSerial.print(IMUS[imu_index].gyro_x);
    bSerial.print(" ");
    bSerial.print(IMUS[imu_index].gyro_y);
    bSerial.print(" ");
    bSerial.print(IMUS[imu_index].gyro_z);
    bSerial.print(" ");
    bSerial.print(IMUS[imu_index].accel_x);
    bSerial.print (" ");
    bSerial.print(IMUS[imu_index].accel_y);
    bSerial.print (" ");
    bSerial.print(IMUS[imu_index].accel_z);
    bSerial.print(" ");
    bSerial.print(IMUS[imu_index].magnetom_x);
    bSerial.print (" ");
    bSerial.print(IMUS[imu_index].magnetom_y);
    bSerial.print (" ");
    bSerial.print(IMUS[imu_index].magnetom_z);
    bSerial.print(" ");
    Serial.print(IMUS[imu_index].time_stamp);
    Serial.print('\n');
  }
  else
  {
    bSerial.print(imu_index);
    bSerial.print(" ");
    bSerial.print(IMUS[imu_index].gyro_z*0); // y
    bSerial.print(" ");
    bSerial.print(IMUS[imu_index].gyro_x*0); // z
    bSerial.print(" ");
    bSerial.print(IMUS[imu_index].gyro_y);
    bSerial.print(" ");
    bSerial.print(IMUS[imu_index].accel_z*0);
    bSerial.print (" ");
    bSerial.print(IMUS[imu_index].accel_x*0);
    bSerial.print (" ");
    bSerial.print(IMUS[imu_index].accel_y);
    bSerial.print(" ");
    bSerial.print(IMUS[imu_index].magnetom_z*0);
    bSerial.print (" ");
    bSerial.print(IMUS[imu_index].magnetom_x*0);
    bSerial.print (" ");
    bSerial.print(IMUS[imu_index].magnetom_y);
    bSerial.print(" ");
    bSerial.print(IMUS[imu_index].time_stamp);
    bSerial.print('\n');
  }
}
