#define ADV_TIMEOUT 20 // 20 sec
void bleout( char* b, int len, int blocklen )
{
  int block;
  for (block = 0; block < len / blocklen; block++)
  {
    bleuart.write(&b[block * blocklen], blocklen);
  }
  bleuart.write(&b[block * blocklen], len % blocklen);
}

void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
     - Enable auto advertising if disconnected
     - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
     - Timeout for fast mode is 30 seconds
     - Start(timeout) with timeout = 0 will advertise forever (until connected)

     For recommended advertising interval
     https://developer.apple.com/library/content/qa/qa1931/_myIndex.html
  */
  Bluefruit.Advertising.restartOnDisconnect(true);
  //Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setInterval(1000, 1000);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(ADV_TIMEOUT);                // 0 = Don't stop advertising after n seconds
}

void configureBLE()
{
  Bluefruit.autoConnLed(false);

  // Config the peripheral connection with maximum bandwidth
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  // Set max power. Accepted values are: -40, -30, -20, -16, -12, -8, -4, 0, 4
  Bluefruit.setTxPower(-30);
  Bluefruit.setName("TopViewEasyStroke");
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.setConnectCallback(BLEconnect_callback);
  Bluefruit.setDisconnectCallback(BLEdisconnect_callback);

  // Configure and Start Device Information Service
  bledis.setManufacturer("IoThingsWare");
  bledis.setModel("TopView IMU Token");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();
  blebas.write(100);
  // Set up and start advertising
  startAdv();
}

void BLEconnect_callback(uint16_t conn_handle)
{
  char central_name[32] = { 0 };
  Bluefruit.Gap.getPeerName(conn_handle, central_name, sizeof(central_name));
  Serial.print("Connected to ");
  Serial.println(central_name);
  
}

void BLEdisconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;
  Serial.println();
  Serial.println("Disconnected");
  NVIC_SystemReset();
}

void adv_stop_callback(void)
{
  startAdv();
}


