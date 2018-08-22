float getBattVoltage()
{
  float batVoltage = analogRead(VBAT); //read voltage
  batVoltage *= 2; //it's devidet by to from resistor devider so multiply it
  batVoltage *= 3.3; //multiplying to 3.3 this is the reference voltage
  return batVoltage /= 1024; //convert from analog read units to voltage and return

}

bool isCriticalPower()
{
  if (getBattVoltage() <= 3.3)
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

float getBattPercents()
{
  return map(getBattVoltage(), 3.2, 4.2, 0, 100);
}

bool braceletNotMoved()
{

}

void ledRegularMode()
{
  // reverse the direction of the fading at the ends of the fade:
  if (current_timer - led_timer_rm >= fade_coef)//fade_coef)
  {
    led_timer_rm = current_timer;

    analogWrite(LED, brightness);
    // change the brightness for next time through the loop:
    brightness = brightness + fade_amount;
    if (brightness <= 0 || brightness >= 240)
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
      led_state = 240;
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
  if (getBattPercents() >= 20)
  {
    ledRegularMode();
  }
  else
  {
    ledLowBatteryMode();
  }
}



