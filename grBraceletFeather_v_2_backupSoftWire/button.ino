/*void buttonClick()
  {
  bttn.poll();
  if (bttn.singleClick())
  {
   // NVIC_SystemReset();
    //something
    Serial.println("SINGLECLICK\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\");
  }
  else if (bttn.doubleClick())
  {
    //enable disable led strip
    NVIC_SystemReset();
    Serial.println("enable IR leds---------------------------------------------");
  }
  else if (bttn.longPress())
  {
    //digitalWrite(LED, HIGH);
    // Serial.println("enable IR leds");
    calibrate();
  }

  }*/

void bttnTick()
{
  bttn.tick();
}

void bttnOnDouble()
{
  Serial.println("double");

}
void bttnOnClick()
{
 // NVIC_SystemReset();
  Serial.println("single");

}

void bttnOnLong()
{
  Serial.println("long");

}
