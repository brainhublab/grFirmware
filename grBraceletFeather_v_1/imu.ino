//#include <LSM6.h> //pololu accelerometer and gyro lib/
//#include <LIS3MDL.h> //magnetometer lib

//LSM6 gyro_acc; //initialization of accelerometer and gyroscope vars
//LIS3MDL mag; //initialization of magnetometer




/*I2C bus */
void checkIfIMUConnected(byte imuId)
{
  Wire.beginTransmission(ACTIVE_ADDR);
  if (Wire.endTransmission() == 0)
  {
    connected_imu_ids[imuId] = 1;
  }

}

void sa0PinsInit()
{

  for (byte i = 0; i < IMUS_NUMBER; i++)
  {

    pinMode(sa0[i], OUTPUT);
    digitalWrite(sa0[i], LOW);
    Serial.println("pins init");

  }
  for (byte i = 0; i < IMUS_NUMBER; i++)
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

void switchIMU(byte imuId)
{
  //Serial.println("switch to imu id");
  if (sa0[imuId - 1] == HIGH && imuId > 0)
  {
    digitalWrite(sa0[imuId - 1], LOW);
  }
  digitalWrite(sa0[imuId], HIGH);
}

void resetSa0()
{
  for (byte i = 0; i < IMUS_NUMBER; i++)
  {
    if (sa0[i] == HIGH)
    {
      digitalWrite(sa0[i], LOW);
    }
  }
}

void resetSa0(byte imu_id)
{
  digitalWrite(sa0[imu_id], LOW);
}

/*initializing gyroscope*/
void gyroInit()
{

  //gyro_acc.init(gyro_acc.device_auto, gyro_acc.sa0_high); //inited in acc
  gyro_acc.writeReg(LSM6::CTRL2_G, 0x3C); // 104 Hz, 2000 dps full scale 4c

}

/*reading data from gyroscope*/
void gyroRead(byte imu_id)
{
  gyro_acc.readGyro(); //make reading from IMU for reference LSM6.h

  /*select array from IMUs and read each axis*/
  IMUS[imu_id].G_AN[0] = gyro_acc.g.x;
  IMUS[imu_id].G_AN[1] = gyro_acc.g.y;
  IMUS[imu_id].G_AN[2] = gyro_acc.g.z;

  /*make reading from gyroscope with offset extraction and IMU sign correction of orientation*/
  if (imu_id == PALM_INDEX) //if its == 5 is palm index on multiplexer
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
  /*
    if (imu_id == 5)
    {
    resetSa0(imu_id);
    }
  */
}

/*initialize accelerometer*/
void accInit()
{

  gyro_acc.init(gyro_acc.device_auto, gyro_acc.sa0_high); //initializing
  gyro_acc.enableDefault(); //enable default flags for both type of data gyro and acc
  gyro_acc.writeReg(LSM6::CTRL1_XL, 0x3C); // 3C 52 Hz, 8 g full scale A0 for 2G or 40

}

/* make some readings from accelerometer */
void accRead(byte imu_id) {
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
void magRead(byte imu_id) {
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

  for (byte i = 0; i < IMUS_NUMBER; i++)
  {
    if (connected_imu_ids[i])
    {
      switchIMU(i);
      accInit();
      gyroInit();
      magInit();
    }
    if (i == 5)
    {
      resetSa0(i);
    }
  }
}

void calibrate()
{

  for (byte imu_id = 0; imu_id < IMUS_NUMBER; imu_id++)
  {
    analogWrite(LED, 240);
    switchIMU(imu_id);

    if (connected_imu_ids[imu_id])
    {
      for (byte calib_iter = 0; calib_iter < 32; calib_iter++ )
      {
        grPrint("Callib iteration: ");
       // grPrint(calib_iter);
        gyroRead(imu_id);
        //delay(20);

        for (uint8_t axsis_id = 0; axsis_id < 3; axsis_id ++)
        {
          IMUS[imu_id].G_AN_OFFSET[axsis_id] += IMUS[imu_id].G_AN[axsis_id];
        }
        delay(20);
      }
      /* extract average from acumulated values for generating offsets*/
      for (uint8_t axsis_id = 0; axsis_id < 3; axsis_id++)
      {
        IMUS[imu_id].G_AN_OFFSET[axsis_id] = IMUS[imu_id].G_AN_OFFSET[axsis_id] / 32;
      }
      //connected_imu_ids[imu_id] = 1; //push true flag in boolean array
      //delay(60);

    }
    if (imu_id == 5)
    {
      resetSa0(imu_id);
    }
    analogWrite(LED, 0);
  }

}

void readIMU()
{
  for (byte i = 0; i < IMUS_NUMBER; i++)
  {
    if (connected_imu_ids[i])
    {
      switchIMU(i);
      gyroRead(i);
      accRead(i);
      magRead(i);
      if (i == 5)
      {
        resetSa0(i);
      }
    }
  }
}

