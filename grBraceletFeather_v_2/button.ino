void buttonClick()
  {
  bttn.poll();
  if (bttn.singleClick())
  {
     //updatePowerMode();
    // NVIC_SystemReset();
    //something
    //updatePowerMode();
    Serial.println("SINGLECLICK\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\");
  }
  else if (bttn.doubleClick())
  {
    //enable disable led strip
    //NVIC_SystemReset();
    Serial.println("enable IR leds---------------------------------------------|||||||||||||||||||||||||||||||||||||");
    delay(20);
  }
  else if (bttn.longPress())
  {
    //digitalWrite(LED, HIGH);
 /*    //Serial.println("Loooooooooooooooooooooooooooooooong");
   analogWrite(LED, 254);
    getDataFlag = false;
    resetSa0();
    //resetBus();
    imuInit();
    calibrate();
    getDataFlag = true;
    analogWrite(LED, 0);

*/
    updatePowerMode();
  }

  }

/*
void buttonClick()
{
  
    buttn.tick();


}

void click1() {
  Serial.println("Button 1 click.");
} // click1


// This function will be called when the button1 was pressed 2 times in a short timeframe.
void doubleclick1() {
  Serial.println("Button 1 doubleclick.");
} // doubleclick1

// This function will be called once, when the button1 is released after beeing pressed for a long time.
void longPress1() {
  Serial.println("Button 1 longPress stop");
  updatePowerMode();
}
*/
