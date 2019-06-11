void initWifi()
{
  WiFi.setPins(8, 7, 4, 2);

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD)
  {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }



}

void tryToConnect()
{
  while (status != WL_CONNECTED)
  {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  server.begin();
  Serial.println("Connected to wifi");

  printWiFiStatus();
  analogWrite(LED, 254);
}

void checkIncomingEvent(WiFiClient* client)
{
   while(client->available()) //check for bugs if freeze change it to if statement
        { // if there's bytes to read from the client,
          char c = client->read();             // read a byte, then
          Serial.write(c);                    // print it out the serial monitor
          if (c == '\n') {                    // if the byte is a newline character
            if (currentLine.length() == 0)
            {
              break;
            }
            else
            { // if you got a newline, then clear currentLine:
              currentLine = "";
            }
          }
          else if (c != '\r')
          { // if you got anything else but a carriage return character,
            currentLine += c;      // add it to the end of the currentLine
          }

          // Check to see if the client request was "ga" - get attributes, "gd" - get data, "sd" - stop data
          if (currentLine.endsWith("ga"))
          {
            analogWrite(LED, 254);
            client->println("GR[L] 123456" );
            analogWrite(LED, 0);

          }
          if (currentLine.endsWith("gd"))
          {
            analogWrite(LED, 0);
            getDataFlag = true;
          }
          if (currentLine.endsWith("sd"))
          {
            digitalWrite(LED, LOW);
            Serial.println("Stop Reading");
            getDataFlag = false;
          }
          if(currentLine.endsWith("calib"))
          {
            calibrationFlag = true;
          }
          if(currentLine.endsWith("susp"))
          {
            powerSaveMode = true;
          }
          
        }
}


void printWiFiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.print(rssi);
  Serial.println(" dBm");

  WiFi.macAddress(mac);
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
