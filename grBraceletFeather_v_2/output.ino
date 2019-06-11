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

void getData()
{
  for (int8_t i = 0; i < IMUS_NUMBER; i++)
  {
    switchIMU(i);
    //checkIfIMUConnected(i);
    readIMU(i);
    //  Serial.print("---|");
    //  Serial.print(i);
    /*
      Serial.print("|---");
      Serial.print(IMUS[i].gyro_x);
      Serial.print(" ");
      Serial.print(IMUS[i].gyro_y);
      Serial.print(" ");
      Serial.print(IMUS[i].gyro_z);
      Serial.print(" ");

      Serial.print(IMUS[i].acc_x);
      Serial.print(" ");
      Serial.print(IMUS[i].acc_y);
      Serial.print(" ");
      Serial.print(IMUS[i].acc_z);
    
*/

  }
  //Serial.println("------------------------| ");


}

void generatePackage()
{
  //getData();
  memset(output_data, 0, sizeof(output_data));

  //output_data[0] = 0x3C;
  int j = 0;
  for (int8_t i = 0; i < 6; i++)
  {
    output_data[j] = (int8_t)connected_imu_ids[i];
    output_data[j + 1] = (int8_t)hb(IMUS[i].acc_x);
    output_data[j + 2] = (int8_t)lb(IMUS[i].acc_x);
    output_data[j + 3] = (int8_t)hb(IMUS[i].acc_y);
    output_data[j + 4] = (int8_t)lb(IMUS[i].acc_y);
    output_data[j + 5] = (int8_t)hb(IMUS[i].acc_z);
    output_data[j + 6] = (int8_t)lb(IMUS[i].acc_z);

    output_data[j + 7] = (int8_t)hb(IMUS[i].gyro_x);
    output_data[j + 8] = (int8_t)lb(IMUS[i].gyro_x);
    output_data[j + 9] = (int8_t)hb(IMUS[i].gyro_y);
    output_data[j + 10] = (int8_t)lb(IMUS[i].gyro_y);
    output_data[j + 11] = (int8_t)hb(IMUS[i].gyro_z);
    output_data[j + 12] = (int8_t)lb(IMUS[i].gyro_z);

    output_data[j + 13] = (int8_t)hb(IMUS[i].mag_x);
    output_data[j + 14] = (int8_t)lb(IMUS[i].mag_x);
    output_data[j + 15] = (int8_t)hb(IMUS[i].mag_y);
    output_data[j + 16] = (int8_t)lb(IMUS[i].mag_y);
    output_data[j + 17] = (int8_t)hb(IMUS[i].mag_z);
    output_data[j + 18] = (int8_t)lb(IMUS[i].mag_z);
    j += 19;
  }
  output_data[114] = (int8_t)currentBatteryLevel;
  output_data[115] = '\r';
  output_data[116] = '\n';

  /* int16_t result = ((output_data[1] & 0xFF) << 8 | (output_data[2] & 0xFF)  ) ;
    Serial.println("|------------------|");

    Serial.println(result);

    Serial.println("------------------");

    Serial.println(IMUS[0].gyro_x);
    Serial.println("|------------------|");
  */
}
