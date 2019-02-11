//#include <LSM6.h> //pololu accelerometer and gyro lib/
//#include <LIS3MDL.h> //magnetometer lib

//LSM6 gyro_acc; //initialization of accelerometer and gyroscope vars
//LIS3MDL mag; //initialization of magnetometer




/*I2C bus */
void checkIfIMUConnected(int8_t imuId)
{
  Wire.beginTransmission(ACTIVE_ADDR);
  if (Wire.endTransmission() == 0)
  {
    connected_imu_ids[imuId] = 1;
  }

}

void sa0PinsInit()
{

  for (int8_t i = 0; i < IMUS_NUMBER; i++)
  {

    pinMode(sa0[i], OUTPUT);
    digitalWrite(sa0[i], LOW);
    Serial.println("pins init");

  }
  for (int8_t i = 0; i < IMUS_NUMBER; i++)
  {
    digitalWrite(sa0[i], HIGH);
    checkIfIMUConnected(i);
    digitalWrite(sa0[i], LOW);
  }

}

void i2cInit()
{
  Wire.begin();
  grPrint("i2cInit");
}

void switchIMU(int8_t imuId)
{
  //Serial.println("switch to imu id");
  /*if(imuId == 0)
  {
    digitalWrite(1, LOW);
    digitalWrite(5, HIGH);
  
  }
  else if(imuId == 4)
  {
    digitalWrite(5, LOW);
    digitalWrite(1, HIGH);
    
  }*/
  if (imuId > 0)
  {
    //Serial.print("IMU IS HIGEST THEN 0:  ");
    //Serial.println(imuId);
    digitalWrite(sa0[imuId - 1], LOW);
  }
  else if (imuId == 0)
  {
    digitalWrite(sa0[5], LOW);
  }
  digitalWrite(sa0[imuId], HIGH);
 
}

void resetSa0()
{
  for (int8_t i = 0; i < IMUS_NUMBER; i++)
  {
    digitalWrite(sa0[i], LOW);
  }
}

void resetSa0(int8_t imu_id)
{
  digitalWrite(sa0[imu_id], LOW);
}

/*initializing gyroscope*/
void gyroInit()
{
  //gyro_acc.init(gyro_acc.device_auto, gyro_acc.sa0_high); //inited in acc
  gyro_acc.writeReg(LSM6::CTRL2_G, 0x4C); // 104 Hz, 2000 dps full scale 4C and 52HZ 3C

}

/*reading data from gyroscope*/
void gyroRead(int8_t imu_id)
{
  gyro_acc.readGyro(); //make reading from IMU for reference LSM6.h

  /*select array from IMUs and read each axis*/
  IMUS[imu_id].G_AN[0] = gyro_acc.g.x;
  IMUS[imu_id].G_AN[1] = gyro_acc.g.y;
  IMUS[imu_id].G_AN[2] = gyro_acc.g.z;

  /*make reading from gyroscope with offset extraction and IMU sign correction of orientation*/
  if (imu_id == PALM_INDEX) //if its == palm index on multiplexer
  {
    IMUS[imu_id].gyro_x = IMU_SIGN[0] * (IMUS[imu_id].G_AN[0] - IMUS[imu_id].G_AN_OFFSET[0]);
    IMUS[imu_id].gyro_y = IMU_SIGN[1] * (IMUS[imu_id].G_AN[1] - IMUS[imu_id].G_AN_OFFSET[1]);
    IMUS[imu_id].gyro_z = IMU_SIGN[2] * (IMUS[imu_id].G_AN[2] - IMUS[imu_id].G_AN_OFFSET[2]);
  }
  else //it's finger
  {
    IMUS[imu_id].gyro_x = FINGER_IMU_SIGN[0] * (IMUS[imu_id].G_AN[0] - IMUS[imu_id].G_AN_OFFSET[0]);
    IMUS[imu_id].gyro_y = FINGER_IMU_SIGN[1] * (IMUS[imu_id].G_AN[1] - IMUS[imu_id].G_AN_OFFSET[1]);
    IMUS[imu_id].gyro_z = FINGER_IMU_SIGN[2] * (IMUS[imu_id].G_AN[2] - IMUS[imu_id].G_AN_OFFSET[2]);
  }

}

/*initialize accelerometer*/
void accInit()
{

 // gyro_acc.init(gyro_acc.device_auto, gyro_acc.sa0_high); //initializing
 gyro_acc.init();
  gyro_acc.enableDefault(); //enable default flags for both type of data gyro and acc
  gyro_acc.writeReg(LSM6::CTRL1_XL, 0x3C); // 3C 52 Hz, 8 g full scale A0 for 2G or 40 may be neet on 104 HZ

}

/* make some readings from accelerometer */
void accRead(int8_t imu_id) {
  gyro_acc.readAcc();
  /* store readed data from imu with 4 bit shifting */
  IMUS[imu_id].A_AN[0] = gyro_acc.a.x >> 4;
  IMUS[imu_id].A_AN[1] = gyro_acc.a.y >> 4;
  IMUS[imu_id].A_AN[2] = gyro_acc.a.z >> 4;

  /*store data from accelrometer with IMU sign correction of orientation*/
  if (imu_id == PALM_INDEX) //if imu_id == 5 its palm
  {
    IMUS[imu_id].acc_x = IMU_SIGN[3] * (IMUS[imu_id].A_AN[0]);
    IMUS[imu_id].acc_y = IMU_SIGN[4] * (IMUS[imu_id].A_AN[1]);
    IMUS[imu_id].acc_z = IMU_SIGN[5] * (IMUS[imu_id].A_AN[2]);
  }
  else
  {
    IMUS[imu_id].acc_x = FINGER_IMU_SIGN[3] * (IMUS[imu_id].A_AN[0]);
    IMUS[imu_id].acc_y = FINGER_IMU_SIGN[4] * (IMUS[imu_id].A_AN[1]);
    IMUS[imu_id].acc_z = FINGER_IMU_SIGN[5] * (IMUS[imu_id].A_AN[2]);
  }


}

/*magnetometer initializing*/

void magInit()
{
  mag.init();
  mag.enableDefault();
}

//*magnetometer reading
void magRead(int8_t imu_id) {
  mag.read();

  if (imu_id == PALM_INDEX)
  {
    IMUS[imu_id].mag_x = IMU_SIGN[6] * mag.m.x;
    IMUS[imu_id].mag_y = IMU_SIGN[7] * mag.m.y;
    IMUS[imu_id].mag_z = IMU_SIGN[8] * mag.m.z;
  }
  else
  {
    IMUS[imu_id].mag_x = FINGER_IMU_SIGN[6] * mag.m.x;
    IMUS[imu_id].mag_y = FINGER_IMU_SIGN[7] * mag.m.y;
    IMUS[imu_id].mag_z = FINGER_IMU_SIGN[8] * mag.m.z;
  }
}


void imuInit()
{

  for (int8_t i = 0; i < IMUS_NUMBER; i++)
  {
    if (connected_imu_ids[i])
    {
      switchIMU(i);
      accInit();
      gyroInit();
      magInit();
    }
  }
  resetSa0();//TODO check if need to reset sa0
}

void calibrate()
{

  for (int8_t imu_id = 0; imu_id < IMUS_NUMBER; imu_id++)
  {
    analogWrite(LED, 220);
    switchIMU(imu_id);

    if (connected_imu_ids[imu_id])
    {
      for (int8_t calib_iter = 0; calib_iter < 32; calib_iter++ )
      {
        grPrint("Callib iteration: ");
        // grPrint(calib_iter);
        gyroRead(imu_id);
        //delay(20);

        for (uint8_t axsis_id = 0; axsis_id < 3; axsis_id ++)
        {
          IMUS[imu_id].G_AN_OFFSET[axsis_id] += IMUS[imu_id].G_AN[axsis_id];
        }
        //delay(20);
      }
      /* extract average from acumulated values for generating offsets*/
      for (uint8_t axsis_id = 0; axsis_id < 3; axsis_id++)
      {
        IMUS[imu_id].G_AN_OFFSET[axsis_id] = IMUS[imu_id].G_AN_OFFSET[axsis_id] / 32;
      }
      //connected_imu_ids[imu_id] = 1; //push true flag in boolean array
      //delay(60);

    }
    analogWrite(LED, 0);
    Serial.println("==============================||=============================||");
    delay(20);
  }

}

void readIMU(int8_t i)
{
  if (connected_imu_ids[i])
  {

    gyroRead(i);
    accRead(i);
    magRead(i);
  }


}

void updateFilter(int8_t imu_id)
{
  if (imu_id == 0)
  {
    IMUS[imu_id].m_filter.update(convertRawGyro(IMUS[imu_id].gyro_x), convertRawGyro(IMUS[imu_id].gyro_y), convertRawGyro(IMUS[imu_id].gyro_z),
                                 convertRawAcceleration(IMUS[imu_id].acc_x), convertRawAcceleration(IMUS[imu_id].acc_y), convertRawAcceleration(IMUS[imu_id].acc_z),
                                 (float)IMUS[imu_id].mag_x, (float)IMUS[imu_id].mag_y, (float)IMUS[imu_id].mag_z);

  }
  else
  {
    IMUS[imu_id].m_filter.update(convertRawGyro(IMUS[imu_id].gyro_x), convertRawGyro(IMUS[imu_id].gyro_y), convertRawGyro(IMUS[imu_id].gyro_z),
                                 convertRawAcceleration(IMUS[imu_id].acc_x), convertRawAcceleration(IMUS[imu_id].acc_y), convertRawAcceleration(IMUS[imu_id].acc_z),
                                 (float)IMUS[imu_id].mag_x, (float)IMUS[imu_id].mag_y, (float)IMUS[imu_id].mag_z);
  }
}

void updateAngles(int8_t imu_id)
{
  roll = (int16_t)IMUS[imu_id].m_filter.getRoll();
  pitch = (int16_t)IMUS[imu_id].m_filter.getPitch();
  yaw = (int16_t)IMUS[imu_id].m_filter.getYaw();
}
int getRelativeAngle(float source, float compare)
{
  int sign = compare > source ? 1 : -1;
  angle = compare - source;
  K = -sign * PI * 2;
  return (int)((abs(K + angle) < abs(angle)) ? K + angle : angle);
}
