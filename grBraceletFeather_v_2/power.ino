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
  return getBattLevel() <= 20;
}

bool enterPowerSaveMode()
{
  return braceletNotMoved();
}

void enterPowerSaveIMU()
{
  for (int8_t imu_id = 0; imu_id < IMUS_NUMBER; imu_id++)
  {
    resetSa0();
    switchIMU(imu_id);

    // disable acc
    gyro_acc.writeReg(LSM6::CTRL1_XL, 0x0C); // disable
    delay(20);

    // disable gyro
    gyro_acc.writeReg(LSM6::CTRL2_G, 0x0C); // disable
    delay(20);

    // disable mag
    mag.writeReg(LIS3MDL::CTRL_REG3, 0x2); // disable
    delay(20);
  }

  IMU_powered_on = false;
  grPrint("imu powered OFF");
}

void exitPowerSaveIMU()
{
  resetSa0();
  imuInit();
}

void enterPowerSaveMcu()
{}

void exitPowerSaveMcu()
{}

bool braceletNotMoved()
{}

void ledOnConnection()
{
  current_timer = millis();

  if (current_timer - led_timer_rm >= 10)
  {
    // grPrint("-----------------------------ONCONNECTION");
    led_timer_rm = current_timer;

    analogWrite(LED, brightness);

    // change the brightness for next time through the loop:
    brightness = brightness + 5;
    if (brightness >= 250)
    {
      brightness = 0;
    }
  }
}
void ledSessionMode()
{
  // reverse the direction of the fading at the ends of the fade:
  current_timer = millis();

  if (current_timer - led_timer_rm >= 40)
  {
    // grPrint("-----------------------------ONSESSIONMODE");
    led_timer_rm = current_timer;

    analogWrite(LED, brightness);

    // change the brightness for next time through the loop:
    brightness = brightness + abs(fade_amount);
    if (brightness >= 250)
    {
      brightness = 0;
    }
  }

}

void ledWaitMode()
{
  // reverse the direction of the fading at the ends of the fade:
  current_timer = millis();
  if ((current_timer - led_timer_rm) >= 100)
  {
    led_timer_rm = current_timer;
    // grPrint("-----------------------------ONWAITMODE");
    // grPrint(brightness);
    // grPrint(fade_amount);
    analogWrite(LED, brightness);
    // change the brightness for next time through the loop:
    brightness += fade_amount;
    if ((brightness == 0) || (brightness > 250))
    {
      fade_amount *= -1; //-fade_amount;
    }
  }
}

void ledLowPowerMode()
{
  current_timer = millis();
  if (current_timer - led_timer_lm >= 20)
  {
    // grPrint("-----------------------------ONLOWPOWERMODE");

    led_timer_lm = current_timer;

    analogWrite(LED, 0);
    if (led_timer_lm - ledLongLow >= 3000)
    {
      ledLongLow = led_timer_lm;
      analogWrite(LED, 250);
    }
  }
}

void ledBlink()
{

  if (sessionMode && !powerSaveMode)
  {
    ledSessionMode();
  }
  else if (waitMode && !powerSaveMode)
  {
    ledWaitMode();
  }
  else if ((powerSaveMode && sessionMode) || (powerSaveMode && waitMode))
  {
    ledLowPowerMode();
  }

}

void updatePowerMode()
{
  powerSaveMode = !powerSaveMode;
  if (powerSaveMode)
  {
    enterPowerSaveIMU();
    WiFi.maxLowPowerMode();
    if (waitMode && status == WL_CONNECTED)
    {
      WiFi.disconnect();
      status = WL_IDLE_STATUS;
    }
  }
  else
  {
    // set higher timeout value while calibrating
    // we still need tha watchdog since calibration
    // uses I2C and can hang
    Watchdog.disable();
    Watchdog.enable(WATCHDOG_SETUP_TIMEOUT);

    exitPowerSaveIMU();
    // delay(20);
    // imuInit();
    calibrate();
    WiFi.noLowPowerMode();

    // and enable it back on
    Watchdog.disable();
    Watchdog.enable(WATCHDOG_LOOP_TIMEOUT);
  }

  //currentBatteryLevel = getBattLevel();
}

uint8_t mapBattary(uint16_t voltage, uint16_t minVoltage, uint16_t maxVoltage)
{
  uint8_t result = 105 - (105 / (1 + pow(1.724 * (voltage - minVoltage) / (maxVoltage - minVoltage), 5.5)));
  return result >= 100 ? 100 : result;
}
