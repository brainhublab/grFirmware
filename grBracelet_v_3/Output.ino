void printdata(uint8_t i)
{
  if(i==5)
  {
    bSerial.print(i);
    bSerial.print(" ");
    bSerial.print(SENSORS[i].gyro_x);
    bSerial.print(" ");
    bSerial.print(SENSORS[i].gyro_y);
    bSerial.print(" ");
    bSerial.print(SENSORS[i].gyro_z);
    bSerial.print(" ");
    bSerial.print(SENSORS[i].accel_x);
    bSerial.print (" ");
    bSerial.print(SENSORS[i].accel_y);
    bSerial.print (" ");
    bSerial.print(SENSORS[i].accel_z);
    bSerial.print(" ");
    bSerial.print(SENSORS[i].magnetom_x);
    bSerial.print (" ");
    bSerial.print(SENSORS[i].magnetom_y);
    bSerial.print (" ");
    bSerial.print(SENSORS[i].magnetom_z);
    bSerial.print(" ");
    bSerial.print(SENSORS[i].time_stamp);
    bSerial.print('\n');
  }
  else
  {
    bSerial.print(i);
    bSerial.print(" ");
    bSerial.print(SENSORS[i].gyro_z*0); // y
    bSerial.print(" ");
    bSerial.print(SENSORS[i].gyro_x*0); // z
    bSerial.print(" ");
    bSerial.print(SENSORS[i].gyro_y);
    bSerial.print(" ");
    bSerial.print(SENSORS[i].accel_z*0);
    bSerial.print (" ");
    bSerial.print(SENSORS[i].accel_x*0);
    bSerial.print (" ");
    bSerial.print(SENSORS[i].accel_y);
    bSerial.print(" ");
    bSerial.print(SENSORS[i].magnetom_z*0);
    bSerial.print (" ");
    bSerial.print(SENSORS[i].magnetom_x*0);
    bSerial.print (" ");
    bSerial.print(SENSORS[i].magnetom_y);
    bSerial.print(" ");
    bSerial.print(SENSORS[i].time_stamp);
    bSerial.print('\n');
  }
}
