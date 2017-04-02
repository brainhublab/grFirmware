void printdata(uint8_t i)
{
      //Serial.print("!");
      Serial.print(i);


      Serial.print(" ");
      Serial.print(SENSORS[i].gyro_x);
      Serial.print(" ");
      Serial.print(SENSORS[i].gyro_y);
      Serial.print(" ");
      Serial.print(SENSORS[i].gyro_z);
      Serial.print(" ");
      Serial.print(SENSORS[i].accel_x);
      Serial.print (" ");
      Serial.print(SENSORS[i].accel_y);
      Serial.print (" ");
      Serial.print(SENSORS[i].accel_z);
      Serial.print(" ");
      Serial.print(SENSORS[i].magnetom_x);
      Serial.print (" ");
      Serial.print(SENSORS[i].magnetom_y);
      Serial.print (" ");
      Serial.print(SENSORS[i].magnetom_z);
      Serial.print("\n");
}
