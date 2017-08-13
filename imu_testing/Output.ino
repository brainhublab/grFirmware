void printdata()
{

  //  bSerial.print(i);
    Serial.print("$");
    Serial.print(SENSOR.gyro_x / 256.0);
      Serial.print(" ");
   // Serial.print(filter(SENSOR.accel_x));
   // Serial.print(";");
    //bSerial.print("$");
    Serial.print(SENSOR.gyro_y  / 256.0);
    Serial.print(" ");
   // Serial.print("$");
    Serial.print(SENSOR.gyro_z  / 256.0);
    Serial.print('\n');
    
    bSerial.print(SENSOR.accel_x);
    bSerial.print (" ");
    bSerial.print(SENSOR.accel_y);
    bSerial.print (" ");
    bSerial.print(SENSOR.accel_z);
    bSerial.print(" ");
    bSerial.print(SENSOR.magnetom_x);
    bSerial.print (" ");
    bSerial.print(SENSOR.magnetom_y);
    bSerial.print (" ");
    bSerial.print(SENSOR.magnetom_z);
    bSerial.print(" ");
    bSerial.print(SENSOR.timestamp);
    bSerial.print('\n');
    
}
