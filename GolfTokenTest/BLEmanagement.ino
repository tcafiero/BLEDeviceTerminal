#include <BLEPeripheral.h>
#include "BLESerial.h"

//custom boards may override default pin definitions with BLESerial(PIN_REQ, PIN_RDY, PIN_RST)
BLESerial bleSerial;

void bleout( char* b, int len, int blocklen )
{
  int block;
  for (block = 0; block < len / blocklen; block++)
  {
    bleSerial.write((const unsigned char*)&b[block * blocklen], blocklen);
    WAIT(20);
  }
  bleSerial.write((const unsigned char*)&b[block * blocklen], len % blocklen);
  bleSerial.flush();
}


void BLEconnect_callback(BLECentral& central) {
  // central connected event handler
  Serial.print(F("Connected event, central: "));
  Serial.println(central.address());
}

void BLEdisconnect_callback(BLECentral& central) {
  // central connected event handler
  Serial.print(F("Disonnected event, central: "));
  Serial.println(central.address());
}

void configureBLE()
{
  bleSerial.setAdvertisingInterval(500);
  bleSerial.setConnectable(true);
  bleSerial.setTxPower(-30);
    // assign event handlers for connected, disconnected to peripheral
  bleSerial.setLocalName("TopViewEasyStroke");
  bleSerial.setEventHandler(BLEConnected, BLEconnect_callback);
  bleSerial.setEventHandler(BLEDisconnected, BLEdisconnect_callback);
  bleSerial.begin();
}



