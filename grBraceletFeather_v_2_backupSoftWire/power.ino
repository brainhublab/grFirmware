/*float getBattVoltage()
{
  float batVoltage = analogRead(VBAT); //read voltage
  batVoltage *= 2; //it's devidet by to from resistor devider so multiply it
  batVoltage *= 3.3; //multiplying to 3.3 this is the reference voltage

  return (batVoltage /= 1024);// -= 2.55; //convert from analog read units to voltage and return

}
*/
bool isCriticalPower()
{
  if (batt.level() <= 10)
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
/*
float getBattPercents()
{
  uint8_t bp = grMap(getBattVoltage(), 3.2, 4.2, 0.0, 100.0);
  if (bp > 100)
  {
    bp = bp - (bp - 100);
  }
  return bp;
}
*/
/*
void powerOffIMU()
{
  //disable acc
  gyro_acc.writeReg(LSM6::CTRL1_XL, 0x0C); // disable
  delay(20);

  //disable gyro
  gyro_acc.writeReg(LSM6::CTRL2_G, 0x0C); //disable
  delay(20);

  //disable mag
  //mag.writeReg(LIS3MDL::CTRL_REG3, 0x2); //disable
  IMU_powered_on = false;
  grPrint("imu powered OFF");


}
*/
/*
void powerOnIMU()
{
  //disable acc
  gyro_acc.enableDefault(); //enable default flags for both type of data gyro and acc
  gyro_acc.writeReg(LSM6::CTRL1_XL, 0x3C); // 3C 52 Hz, 8 g full scale A0 for 2G or 40 may be neet on 104 HZ
  delay(20);
  //disable gyro
  gyro_acc.writeReg(LSM6::CTRL2_G, 0x3C); // 104 Hz, 2000 dps full scale 4C and 52HZ 3C
  delay(20);

  // mag.enableDefault();
  delay(20);

  IMU_powered_on = true;
  grPrint("imu powered ON");


}
*/
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
  /*if (getBattPercents() >= 20)
  {
    ledRegularMode();
  }
  else
  {
    ledLowBatteryMode();
  }*/
}

uint8_t grMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (uint8_t)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}
