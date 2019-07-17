void pollAll()
{
  // poll in function in case new buttons are added
  bttn.poll();
}

// Used in interrupt callback to set button flags
void buttonActions()
{
  if (bttn.longPress()) {
    LONG_PRESS = true;
  } else if (bttn.doubleClick()) {
    DOUBLE_CLICK = true;
  } else if (bttn.singleClick()) {
    SINGLE_CLICK = true;
  }
}

// Actual handling of button events
void handleBtn()
{
  // check for bttn input
  if (LONG_PRESS) {
    Serial.println("long press");
    LONG_PRESS = false;
    updatePowerMode();
  } else if (DOUBLE_CLICK) {
    Serial.println("double click");
    DOUBLE_CLICK = false;
    NVIC_SystemReset();
    delay(20);
  } else if (SINGLE_CLICK) {
    Serial.println("single click");
    SINGLE_CLICK = false;
    sessionMode = !sessionMode;
    waitMode = !waitMode;
  }
}
