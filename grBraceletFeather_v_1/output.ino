void grPrint(String output)
{
 if (SERIAL_VERBOSE_MODE)
  {
    Serial.println(output);
  }
}

// A small helper
void error(const __FlashStringHelper*err) 
{
  Serial.println(err);
  while (1);
}
