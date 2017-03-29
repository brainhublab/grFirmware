void printdata(uint8_t i)
{
      Serial.print("!");
      Serial.print(i);

      Serial.print(",AN:");
      Serial.print(SENSORS[i].AN[0]);
      Serial.print(",");
      Serial.print(SENSORS[i].AN[1]);
      Serial.print(",");
      Serial.print(SENSORS[i].AN[2]);
      Serial.print(",");
      Serial.print(SENSORS[i].AN[3]);
      Serial.print (",");
      Serial.print(SENSORS[i].AN[4]);
      Serial.print (",");
      Serial.print(SENSORS[i].AN[5]);
      Serial.print(",");
      Serial.print(SENSORS[i].magnetom_x);
      Serial.print (",");
      Serial.print(SENSORS[i].magnetom_y);
      Serial.print (",");
      Serial.print(SENSORS[i].magnetom_z);
      Serial.println();
}
