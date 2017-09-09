void printdata(uint8_t i)
{
  if(i==0)
  {
    Serial.print(i);
    Serial.print(" ");
    Serial.print(SENSORS[i].gyro_x);
    Serial.print(" ");
    Serial.print(SENSORS[i].gyro_y);
    Serial.print(" ");
    Serial.print(SENSORS[i].gyro_z);
    // Serial.print(" ");
    //Serial.print(SENSORS[i].accel_x);
    //Serial.print (" ");
    //Serial.print(SENSORS[i].accel_y);
    //Serial.print (" ");
    //Serial.print(SENSORS[i].accel_z);
    // Serial.print(" ");
    // Serial.print(SENSORS[i].magnetom_x);
    // Serial.print (" ");
    // Serial.print(SENSORS[i].magnetom_y);
    // Serial.print (" ");
    // Serial.print(SENSORS[i].magnetom_z);
    // Serial.print(" ");
    // Serial.print(SENSORS[i].timestamp);
    Serial.print('\n');
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
    bSerial.print(SENSORS[i].timestamp);
    bSerial.print('\n');
  }
}
