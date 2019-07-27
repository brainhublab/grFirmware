void initWifi()
{
  WiFi.setPins(8, 7, 4, 2);

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD)
  {
    grPrint("WiFi shield not present");
    // don't continue:
    while (true);
  }

  WiFi.maxLowPowerMode();
}

void tryToConnect()
{
  int8_t tryToConnect = 0;
  while (status != WL_CONNECTED && tryToConnect <= connectionAttempts)
  {
    if (SERIAL_VERBOSE_MODE)
    {
      Serial.print("Attempting to connect to SSID: ");
      Serial.println(ssid);
    }

    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    unsigned long onConTimer = millis();
    while (millis() - onConTimer <= 10000) // wait 10 seconds for connection:
    {
      grPrint(millis() - onConTimer );
      ledOnConnection();
    }

    tryToConnect ++;
  }

  server.begin();
  grPrint("Connected to wifi");

  printWiFiStatus();
}

void checkIncomingEvent(WiFiClient* client)
{
  while (client->available()) //check for bugs if freeze change it to if statement
  {
    char c = client->read(); // read a byte, then
    grPrint(c); // print it out the serial monitor

    if (c == '\n') {
      // if you got a newline, then clear currentLine
      if (currentLine.length() != 0)
      {
        currentLine = "";
      }
    }
    else if (c != '\r')
    { // if you got anything else but a carriage return character,
      currentLine += c; // add it to the end of the currentLine
    }

    // Check to see if the client request was "ga" - get attributes, "gd" - get data, "sd" - stop data
    if (currentLine.endsWith("ga"))
    {
      client->println("GR[L] 123456" );
    }

    if (currentLine.endsWith("gd"))
    {
      getDataFlag = true;
    }

    if (currentLine.endsWith("sd"))
    {
      digitalWrite(LED, LOW);
      grPrint("Stop Reading");
      getDataFlag = false;
    }

    if (currentLine.endsWith("calib"))
    {
      calibrationFlag = true;
      calibrate();
    }

    if (currentLine.endsWith("susp"))
    {
      powerSaveMode = true;
    }
  }
}


void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  if (SERIAL_VERBOSE_MODE)
  {
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
  }

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  if (SERIAL_VERBOSE_MODE)
  {
    Serial.print("IP Address: ");
    Serial.println(ip);
  }

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  if (SERIAL_VERBOSE_MODE)
  {
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");
  }

  WiFi.macAddress(mac);
  if (SERIAL_VERBOSE_MODE)
  {
    Serial.print("MAC: ");
    Serial.print(mac[5], HEX);
    Serial.print(":");
    Serial.print(mac[4], HEX);
    Serial.print(":");
    Serial.print(mac[3], HEX);
    Serial.print(":");
    Serial.print(mac[2], HEX);
    Serial.print(":");
    Serial.print(mac[1], HEX);
    Serial.print(":");
    Serial.println(mac[0], HEX);
  }
}
