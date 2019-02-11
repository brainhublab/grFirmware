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

void generatePackage(int8_t imu_id)
{
  /*
    IMUS[imu_id].gyro_x = 12345;
    IMUS[imu_id].gyro_y = 12345;
    IMUS[imu_id].gyro_z = 12345;

    IMUS[imu_id].acc_x = 12345;
    IMUS[imu_id].acc_y = 12345;
    IMUS[imu_id].acc_z = 12345;

    IMUS[imu_id].mag_x = 12345;
    IMUS[imu_id].mag_y = 12345;
    IMUS[imu_id].mag_z = 12345;
  */
  //memset(output_data, 0, sizeof(output_data));
  memset(packet_data, 0, sizeof(output_data));

  if (imu_id == 0)
  {
    
    palmQ.a = IMUS[imu_id].m_filter.q0;
    palmQ.b = IMUS[imu_id].m_filter.q1;
    palmQ.c = IMUS[imu_id].m_filter.q2;
    palmQ.d = IMUS[imu_id].m_filter.q3;



    last_palm_yaw = yaw;
    last_palm_roll = roll;
    last_palm_pitch = pitch;
/*
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

*/


  }
  else if (imu_id == 4)
  {
    /*uint8_t uroll = (uint8_t) (roll < 0 ? -roll : roll);
      uint8_t relative_pitch = pitch - last_palm_pitch;
      int8_t upitch = (int8_t) (relative_pitch < 0 ? -relative_pitch : relative_pitch);
      uint8_t relative_yaw =  yaw - last_palm_yaw;
      uint8_t uyaw = (uint8_t) (relative_yaw < 0 ? -relative_yaw : relative_yaw);
      sign_arr[imu_id] = (relative_yaw > 0); */



    if (imu_id == 4)
    {
/*
      Quaternion fing;
      indexQ.a = IMUS[imu_id].m_filter.q0;
      indexQ.b = IMUS[imu_id].m_filter.q1;
      indexQ.c = IMUS[imu_id].m_filter.q2;
      indexQ.d = IMUS[imu_id].m_filter.q3;

     // Quaternion fin = inverseQ() *= indexQ;
      Quaternion fin = indexQ.rotation_between_vectors(palmQ);
      //fin.normalize();
      float qroll = atan2f(fin.a * fin.b + fin.c * fin.d, 0.5f - fin.b * fin.b - fin.c * fin.c);
      float qpitch = asinf(-2.0f * (fin.b * fin.d - fin.a * fin.c));
      float qyaw = atan2f(fin.b * fin.c + fin.a * fin.d, 0.5f - fin.c * fin.c - fin.d * fin.d);
      qroll  *= 57.29578f;
      qpitch *= 57.29578f;
      qyaw = qyaw * 57.29578f + 180.0f;

*/

      int relPitch = getRelativeAngle( last_palm_pitch,  pitch);
      // Serial.println(last_palm_pitch);
      Serial.print("Orientation: ");
      Serial.print(yaw); //yaw
      Serial.print(" ");
      Serial.print(relPitch); //pitch
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


  //   output_data[0] =
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
Quaternion inverseQ()
{

  Quaternion tmp  = palmQ.conj();
  return tmp *= (1 / palmQ.norm());
}
