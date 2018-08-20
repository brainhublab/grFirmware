float getBatVoltage()
{
  float batVoltage = analogRead(VBAT); //read voltage
  batVoltage *= 2; //it's devidet by to from resistor devider so multiply it
  batVoltage *= 3.3; //multiplying to 3.3 this is the reference voltage

  return batVoltage /= 1024; //convert from analog read units to voltage and return

}

bool isCriticalPower()
{
  if (getBatVoltage() <= 3.3)
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

bool braceletNotMoved()
{
  
}


