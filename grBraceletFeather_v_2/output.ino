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
  for (int i = 0; i < 108; i++)
  {
    output_data[i] = 127;
  }
}

int16_t dat = 12345;
byte con = 1;


void generatePackage(int8_t imu_id)
{

  memset(output_data, 0, sizeof(output_data));
  snprintf(output_data, sizeof(output_data), "%d%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%d%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%d%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%d%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%d%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%d%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c",
           1, highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), highByte(dat), lowByte(dat),
           highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), highByte(dat), lowByte(dat),
           highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), '/',
           2, highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), highByte(dat), lowByte(dat),
           highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), highByte(dat), lowByte(dat),
           highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), '/',
           3, highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), highByte(dat), lowByte(dat),
           highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), highByte(dat), lowByte(dat),
           highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), '/',
           4, highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), highByte(dat), lowByte(dat),
           highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), highByte(dat), lowByte(dat),
           highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), '/',
           5, highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), highByte(dat), lowByte(dat),
           highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), highByte(dat), lowByte(dat),
           highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), '/',
           6, highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), highByte(dat), lowByte(dat),
           highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), highByte(dat), lowByte(dat),
           highByte(dat), lowByte(dat), highByte(dat), lowByte(dat), highByte(dat), lowByte(dat));


 
   /* int16_t result = ((output_data[1] & 0xFF) << 8 | (output_data[2] & 0xFF)  ) ;
    Serial.println("|------------------|");
 
    Serial.println(result);
  
    Serial.println("------------------");
      
    Serial.println(IMUS[imu_id].gyro_x);
    Serial.println("|------------------|");
  */

  // "%x%d%d%d%d%d%d%d%d%d%d%d%d"   "%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d"  %d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%d%c
  //"%c%c%c%c%c%c%c%c%c%c%c%c%c"    "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c"

}
