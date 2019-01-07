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
  setupGatt();


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

void initCharacteristics()
{

    sensorServiceUUID[3] += 0x01; //prosto edenica i troika
    Serial.println("- adding characteristic IMU GATT");
    sensorServiceId = gatt.addCharacteristic(sensorServiceUUID, GATT_CHARS_PROPERTIES_READ | GATT_CHARS_PROPERTIES_NOTIFY, 1, 20, BLE_DATATYPE_BYTEARRAY);//BLE_DATATYPE_STRING BLE_DATATYPE_BYTEARRAY


  if (sensorServiceId == 0)
  {
    error(F("Failed to init characteristic"));
  }

}


void setupGatt()
{
  //ble.setInterCharWriteDelay(5);

  Serial.println("Adding sensor service");
  sensorServiceId = gatt.addService(sensorServiceUUID);
  if (sensorServiceId == 0)
  {
    Serial.println(sensorServiceId);
    ble.atcommand("AT+GATTLIST");
    error(F("Failed to create sensors service "));
  }
  ble.atcommand("AT+GATTLIST");

  initCharacteristics();

  // Serial.println("Adding sensors service UUID to the advertising payload");
  // uint8_t advdata[] { 0x02, 0x01, 0x06, 0x11, 0x06, 0x6d, 0x2f, 0x1c, 0x2a, 0xe3, 0x1d, 0x0d, 0xb5, 0xea, 0x45, 0x15, 0xc0, 0x08, 0x64, 0xfc };
  // ble.setAdvData(advdata, sizeof(advdata));

  Serial.println("Performing SW reset (service changes require reset)");
  ble.reset();
}

bool BLEConnected()
{
  if (ble.isConnected())
  {
    conn_flag = true;
    return true;
  }
  else
  {
    //delay(500);
    return false;
    conn_flag = false;
  }
}

void setAttributes()
{

  ble.sendCommandCheckOK(dev_name.c_str());
  ble.sendCommandCheckOK("AT+GATTCLEAR");
  ble.reset();
}
