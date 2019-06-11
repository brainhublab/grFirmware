uint16_t getBattVoltage()
{
  analogRead(VBAT);
  delay(2);
  return (uint16_t)analogRead(VBAT) * 2 * 3300 / 1024;
}


uint8_t getBattLevel()
{
  uint16_t voltage = getBattVoltage();
  if (voltage <= minBatteryVoltage)
  {
    return 0;
  }
  else if (voltage >= maxBatteryVoltage)
  {
    return 100;
  }
  else
  {
    return mapBattary(voltage, minBatteryVoltage, maxBatteryVoltage);
  }
}


bool isCriticalPower()
{
  if (getBattLevel() <= 20)
  {
    return true;
  }
  return false;
}

bool eneterPowerSaveMode()
{
  if (braceletNotMoved)
  {
    return true;
  }
  return false;
}


void enterPowerSaveIMU()
{
  for (int8_t imu_id = 0; imu_id < IMUS_NUMBER; imu_id++)
  {
    resetSa0();
    switchIMU(imu_id);
    //disable acc
    gyro_acc.writeReg(LSM6::CTRL1_XL, 0x0C); // disable
    delay(20);

    //disable gyro
    gyro_acc.writeReg(LSM6::CTRL2_G, 0x0C); //disable
    delay(20);
    //disable mag
    mag.writeReg(LIS3MDL::CTRL_REG3, 0x2); //disable
    delay(20);
  }



  IMU_powered_on = false;
  grPrint("imu powered OFF");


}


void exitPowerSaveIMU()
{
  resetSa0();
  imuInit();
  /*
    for (int8_t imu_id = 0; imu_id < IMUS_NUMBER; imu_id++)
    {
    resetSa0();
    switchIMU(imu_id);
    //disable acc
    gyro_acc.init(LSM6::device_auto, LSM6::sa0_high);
    gyro_acc.enableDefault(); //enable default flags for both type of data gyro and acc
    gyro_acc.writeReg(LSM6::CTRL1_XL, 0x3C); // 3C 52 Hz, 8 g full scale A0 for 2G or 40 may be neet on 104 HZ
    delay(20);
    //disable gyro
    gyro_acc.writeReg(LSM6::CTRL2_G, 0x4C); // 104 Hz, 2000 dps full scale 4C and 52HZ 3C
    delay(20);
    mag.enableDefault();
    delay(20);
    }


    IMU_powered_on = true;
    grPrint("imu powered ON");
  */

}

void enterPowerSaveMcu()
{

}

void exitPowerSaveMcu()
{

}
bool braceletNotMoved()
{

}

void ledRegularMode()
{
  // reverse the direction of the fading at the ends of the fade:
  if (current_timer - led_timer_rm >= fade_coef)
  {
    led_timer_rm = current_timer;

    analogWrite(LED, brightness);
    // change the brightness for next time through the loop:
    brightness = brightness + fade_amount;
    if (brightness <= 0 || brightness >= 220)
    {
      fade_amount = -fade_amount;
    }
  }
  //delay(30);
}
void ledLowBatteryMode()
{
  if (current_timer - led_timer_lm >= led_blink)//led_blink);
  {
    led_timer_lm = current_timer;

    if (led_state == 0)
    {
      led_state = 220;
    }
    else
    {
      led_state = 0;
    }
    analogWrite(LED, led_state);
  }

}
void ledBlink()
{
  if (getBattLevel() >= 20)
  {
    ledRegularMode();
  }
  else
  {
    ledLowBatteryMode();
  }
}

void updatePowerMode()
{
  powerSaveMode = !powerSaveMode;
  if (powerSaveMode)
  {
    enterPowerSaveIMU();
    WiFi.maxLowPowerMode();
    /* while (powerSaveMode)
      {
       analogWrite(LED, 0);
        delay(3000);
       //Watchdog.sleep(2000);
       analogWrite(LED, 250);
       //       buttonClick();
       delay(60);
      }*/



  }
  else
  {
    exitPowerSaveIMU();
    // delay(20);
    //  imuInit();
    calibrate();
    WiFi.noLowPowerMode();
    //client.flush();
    /*
      #ifdef USBCON
        USBDevice.attach();
      #endif
    */
  }

  //currentBatteryLevel = getBattLevel();
}
uint8_t mapBattary(uint16_t voltage, uint16_t minVoltage, uint16_t maxVoltage)
{
  uint8_t result = 105 - (105 / (1 + pow(1.724 * (voltage - minVoltage) / (maxVoltage - minVoltage), 5.5)));
  return result >= 100 ? 100 : result;
}
