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

int8_t bitArrayToInt8(bool arr[], int count)
{
  int ret = 0;
  int tmp;
  for (int i = 0; i < count; i++) {
    tmp = arr[i];
    ret |= tmp << (count - i - 1);
  }
  return ret;
}

void genPack()
{
  for(int i=0;i< 108; i++)
  {
    output_data[i] = 127;
  }
}

void generatePackage(int8_t imu_id)
{

  //memset(output_data, 0, sizeof(output_data));
  memset(packet_data, 0, sizeof(output_data));

  if (imu_id == 0)
  {
    




    last_palm_yaw = yaw;
    last_palm_roll = roll;
    last_palm_pitch = pitch;

    packet_data[7] = hb(yaw);
    packet_data[8] = lb(yaw);

    packet_data[9] = hb(pitch);
    packet_data[10] = lb(pitch);

    packet_data[11] = hb(roll);
    packet_data[12] = lb(roll);

    packet_data[13] = hb(IMUS[imu_id].acc_x);
    packet_data[14] = lb(IMUS[imu_id].acc_x);
    packet_data[15] = hb(IMUS[imu_id].acc_y);
    packet_data[16] = lb(IMUS[imu_id].acc_y);
    packet_data[17] = hb(IMUS[imu_id].acc_z);
    packet_data[18] = lb(IMUS[imu_id].acc_z);

  }
  else if (imu_id == 4)
  {




    if (imu_id == 4)
    {

      int relPitch = getRelativeAngle( last_palm_pitch,  pitch);
      // Serial.println(last_palm_pitch);
      Serial.print("Orientation: ");
      Serial.print(yaw); //yaw
      Serial.print(" ");
      Serial.print(pitch); //pitch
      Serial.print(" ");
      Serial.print(roll); //roll
      Serial.print(" ");
      Serial.print(last_palm_yaw); //pyaw
      Serial.print(" ");
      Serial.print(last_palm_pitch); //ppitch
      Serial.print(" ");
      Serial.println(last_palm_roll); //proll

    }

    // Serial.print(last_palm_yaw);
    // Serial.print(" ");
    //Serial.print(uroll);
    // Serial.println();

    //    packet_data[imu_id + 1] = uyaw;

    if (imu_id == 5)
    {
      packet_data[0] = 1; // from 0 to 255
      packet_data[1] = bitArrayToInt8(sign_arr, 8);
      // send
      // gatt.setChar(sensorServiceId, packet_data, 19);

      // Serial.println("Data is Sended");
      end_timer = millis();
      unsigned long  tmp = end_timer - start_timer;
      // Serial.print(" ");
      // Serial.println(tmp);
    }




  }
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
