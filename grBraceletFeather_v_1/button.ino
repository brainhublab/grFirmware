void buttonClick()
{
  bttn.poll();
  /*if(bttn.pushed())
    {
    //something
    Serial.println("SINGLECLICK\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\");
    }
    else */if (bttn.doubleClick())
  {
    //enable disable led strip
    Serial.println("enable IR leds---------------------------------------------");
  }
  else if (bttn.longPress())
  {
    //digitalWrite(LED, HIGH);
    // Serial.println("enable IR leds");
    calibrate();
  }

}


