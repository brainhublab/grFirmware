//#include <LSM6.h> //pololu accelerometer and gyro lib/
//#include <LIS3MDL.h> //magnetometer lib

//LSM6 gyro_acc; //initialization of accelerometer and gyroscope vars
//LIS3MDL mag; //initialization of magnetometer




/*I2C bus */
void checkIfIMUConnected(int8_t imuId)
{
  //unsigned long timertrs = millis();
//  I2c.beginTransmission(ACTIVE_ADDR);
  delay(3);
  // Serial.println("|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||");
//  byte err = I2c.endTransmission();
byte err = 0;
  if (err == 0)
  {
    if (disconnected_imu_ids[imuId] == 1)
    {
      // while(Wire.endTransmission() != 0)
      // Serial.println("In check Before INIT");

      singleImuInit(); //need ti fix bug with unsecure connection in init function just double check for transmission
      // delay(5);
      disconnected_imu_ids[imuId] = 0;
    }
    connected_imu_ids[imuId] = 1;
  }
  else if (err != 0)
  {
    connected_imu_ids[imuId] = 0;
    disconnected_imu_ids[imuId] = 1;
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
 // Wire.begin();
// I2c.setTimeout_ms(40);
 //I2c.begin();
 //I2c.setTimeout_ms(5);
 //Wire.begin();
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
  if (imuId > 0 )
  {
    //Serial.print("IMU IS HIGEST THEN 0:  ");
    //Serial.println(imuId);
    digitalWrite(sa0[imuId - 1], LOW);
  }
  else if (imuId == 0 )
  {
    digitalWrite(sa0[5], LOW);
  }
  digitalWrite(sa0[imuId], HIGH);
  //delay(1);

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
/*initialize accelerometer*/
void accInit()
{

  // gyro_acc.init(gyro_acc.device_auto, gyro_acc.sa0_high); //initializing
  //Wire.beginTransmission(ACTIVE_ADDR);
  // if (connected_imu_ids[imu_id] == 0)//Wire.endTransmission() == 0)
  //{
  //gyro_acc.init();
 // gyro_acc.init(LSM6::device_auto, LSM6::sa0_high);
 // gyro_acc.enableDefault(); //enable default flags for both type of data gyro and acc
 // gyro_acc.writeReg(LSM6::CTRL1_XL, 0x3C); // 3C 52 Hz, 8 g full scale A0 for 2G or 40 may be neet on 104 HZ
//  lsm6init(lsm6device_auto, sa0_high);
 // lsm6enableDefault();
  //lsm6writeReg(CTRL1_XL, 0x3C);
  //gyro_acc.setTimeout(10);
  //}



}

/* make some readings from accelerometer */
void accRead(int8_t imu_id) {
  //Wire.beginTransmission(ACTIVE_ADDR);
 // checkIfIMUConnected(imu_id);
  if (connected_imu_ids[imu_id] == 1)//Wire.endTransmission() == 0)
  {
    //Serial.println(gyro_acc.last_status);
    //gyro_acc.readAcc();
//    lsm6readAcc();
    /* store readed data from imu with 4 bit shifting */
    IMUS[imu_id].A_AN[0] = a.x >> 4;
    IMUS[imu_id].A_AN[1] = a.y >> 4;
    IMUS[imu_id].A_AN[2] = a.z >> 4;

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




}
/*fill not connected imu with zeros*/
void accSetEmpty(int8_t imu_id)
{
  IMUS[imu_id].acc_x = 255;
  IMUS[imu_id].acc_y = 255;
  IMUS[imu_id].acc_z = 255;
}
/*initializing gyroscope*/
/*
void gyroInit()
{
  //gyro_acc.init(gyro_acc.device_auto, gyro_acc.sa0_high); //inited in acc
  // Wire.beginTransmission(ACTIVE_ADDR);
  //if (connected_imu_ids[imu_id] == 0)//Wire.endTransmission() == 0)
  //{
  gyro_acc.writeReg(LSM6::CTRL2_G, 0x4C); // 104 Hz, 2000 dps full scale 4C and 52HZ 3C

  //}

}
*/
/*reading data from gyroscope*/
/*
void gyroRead(int8_t imu_id)
{
  //Wire.beginTransmission(ACTIVE_ADDR);
 // checkIfIMUConnected(imu_id);
  if (connected_imu_ids[imu_id] == 1)//Wire.endTransmission() == 0)
  {
    gyro_acc.readGyro(); //make reading from IMU for reference LSM6.h
    //select array from IMUs and read each axis
    IMUS[imu_id].G_AN[0] = gyro_acc.g.x;
    IMUS[imu_id].G_AN[1] = gyro_acc.g.y;
    IMUS[imu_id].G_AN[2] = gyro_acc.g.z;

    //make reading from gyroscope with offset extraction and IMU sign correction of orientation
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




}
*/
/*fill gyro data with zeros*/
void gyroSetEmpty(int8_t imu_id)
{
  IMUS[imu_id].gyro_x = 255;
  IMUS[imu_id].gyro_y = 255;
  IMUS[imu_id].gyro_z = 255;
}



/*magnetometer initializing*/

void magInit()
{
  //Wire.beginTransmission(ACTIVE_ADDR);
  //  if (connected_imu_ids[imu_id] == 0)//Wire.endTransmission() == 0)
  //{
  //mag.init(LIS3MDL::device_auto, LIS3MDL::sa1_high);
  //mag.enableDefault();
  lis3mdlinit(lis3mdldevice_auto, sa1_high);
  lis3mdlenableDefault();
  //gyro_acc.setTimeout(10);

  // }

}

//*magnetometer reading
void magRead(int8_t imu_id)
{
  //Wire.beginTransmission(ACTIVE_ADDR);
 // checkIfIMUConnected(imu_id);
  if (connected_imu_ids[imu_id] == 1)//Wire.endTransmission() == 0)
  {
   // mag.read();
   lis3mdlread();
    if (imu_id == PALM_INDEX)
    {
      IMUS[imu_id].mag_x = IMU_SIGN[6] * lis3mdlm.x;
      IMUS[imu_id].mag_y = IMU_SIGN[7] * lis3mdlm.y;
      IMUS[imu_id].mag_z = IMU_SIGN[8] * lis3mdlm.z;
    }
    else
    {
      IMUS[imu_id].mag_x = FINGER_IMU_SIGN[6] * lis3mdlm.x;
      IMUS[imu_id].mag_y = FINGER_IMU_SIGN[7] * lis3mdlm.y;
      IMUS[imu_id].mag_z = FINGER_IMU_SIGN[8] * lis3mdlm.z;
    }
  }


}

/*fill magnetometer data with zeros*/
void magSetEmpty(int8_t imu_id)
{
  IMUS[imu_id].mag_x = 255;
  IMUS[imu_id].mag_y = 255;
  IMUS[imu_id].mag_z = 255;
}

void imuInit()
{
  Serial.println("In imu init");

  for (int8_t i = 0; i < IMUS_NUMBER; i++)
  {
    if (connected_imu_ids[i])
    {
      switchIMU(i);
      delay(10);
      accInit();
      delay(10);

   //   gyroInit();
      delay(10);

      magInit();
      delay(10);
    }
     Serial.println("In imu init1");
  }
  resetSa0();//TODO check if need to reset sa0
}

void singleImuInit()
{
  //delay(1);
  //Serial.println("In Single Init ACC");

  accInit();
  //Serial.println("In Single Init GYGO");
//  gyroInit();
  //Serial.println("In Single Init MAG");
  magInit();
}

void calibrate()
{


  for (int8_t imu_id = 0; imu_id < IMUS_NUMBER; imu_id++)
  {
    analogWrite(LED, 220);
    switchIMU(imu_id);
    checkIfIMUConnected(imu_id);
    if (connected_imu_ids[imu_id] == 1)
    {
      for (int8_t calib_iter = 0; calib_iter < 32; calib_iter++ )
      {
        grPrint("Callib iteration: ");
        // grPrint(calib_iter);
//        gyroRead(imu_id);
        //delay(20);

        for (int8_t axsis_id = 0; axsis_id < 3; axsis_id ++)
        {
          IMUS[imu_id].G_AN_OFFSET[axsis_id] += IMUS[imu_id].G_AN[axsis_id];
        }
        //delay(20);
      }
      /* extract average from acumulated values for generating offsets*/
      for (int8_t axsis_id = 0; axsis_id < 3; axsis_id++)
      {
        IMUS[imu_id].G_AN_OFFSET[axsis_id] = IMUS[imu_id].G_AN_OFFSET[axsis_id] / 32;
      }
      //connected_imu_ids[imu_id] = 1; //push true flag in boolean array
      //delay(60);

    }
    /* else
      {
       for (int8_t axis_id = 0; axis_id < 3; axis_id++)
       {
         IMUS[imu_id].G_AN_OFFSET[axis_id] = 0;
       }
      }*/
    analogWrite(LED, 0);
    Serial.println("==============================||=============================||");
    delay(20);
  }

}

void readIMU(int8_t i)
{
  checkIfIMUConnected(i);
  if (connected_imu_ids[i] == 1)
  {
    //Serial.println("ReadIMU ACC");
   // accRead(i);
    // delay(1);
    //Serial.println("ReadIMU GYRO");

    //gyroRead(i);
    //  delay(1);
    //Serial.println("ReadIMU MAG");

    magRead(i);
    //delay(1);
  }
  else
  {
    accSetEmpty(i);
    gyroSetEmpty(i);
    magSetEmpty(i);
  }
}
