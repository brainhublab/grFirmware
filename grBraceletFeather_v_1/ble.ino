void initBLE()
{
  grPrint("Initialising the Bluefruit LE module :0 : ");
  if (!ble.begin(VERBOSE_MODE))
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));

  }

  grPrint("OK");

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    grPrint("Performing a factory reset: ");
    if ( ! ble.factoryReset() )
    {
      error(F("Couldn't factory reset"));
    }
    if (SET_ATTRIBUTES)
    {
      
      setAttributes();
    }
 
  }

     
    

  ble.echo(false);

  grPrint("Requesting Bluefruit info: ");

  if (SERIAL_VERBOSE_MODE)
  {
    ble.info();
  }

  ble.verbose(false);  // debug info is a little annoying after this point!


  //This wait while the connection with the device is established

  grPrint("****************************** Connected");

  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    grPrint("Change LED activity to " MODE_LED_BEHAVIOUR);
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set module to DATA mode
  grPrint("Switching to DATA mode!");
  ble.setMode(BLUEFRUIT_MODE_DATA);

  grPrint("******************************");


}

bool BLEConnected()
{
  if (ble.isConnected())
  {
    return true;
  }
  else
  {
    delay(200);
    return false;
  }
}

void setAttributes()
{

  ble.sendCommandCheckOK(devName.c_str());
  ble.sendCommandCheckOK("AT+GATTCLEAR");
  ble.reset();
}

