#include <SPI.h>
#include <Adb.h>

int verbosity=1;

boolean phoneIsConnected;
unsigned long lastPhoneUpdateTimeMS;

// Adb connection.
Connection *connectionReceive, *connectionSend;
// Event handler for the shell connection. 
void adbEventHandler(Connection *connection, adb_eventType event, uint16_t length, uint8_t *data)
{
  if(event == ADB_CONNECT)
    Serial.println("ADB_CONNECT");  
  else if(event == ADB_DISCONNECT)
  {    
    phoneIsConnected = false;
    Serial.println("ADB_DISCONNECT");  
  }
  else if(event == ADB_CONNECTION_OPEN)
  {    
    Serial.println("ADB_CONNECTION_OPEN");  
    phoneIsConnected = true;
    lastPhoneUpdateTimeMS = millis();
  }
  else if(event == ADB_CONNECTION_CLOSE)
  {    
    phoneIsConnected = false;    
    Serial.println("ADB_CONNECTION_CLOSE");
  }
  else if(event == ADB_CONNECTION_FAILED)
  {    
    phoneIsConnected = false;  
    Serial.println("ADB_CONNECTION_FAILED");
  }
  else if (event == ADB_CONNECTION_RECEIVE)
  {
    Serial.println("Got some data");
    lastPhoneUpdateTimeMS = millis();
  }
}

void setup()
{
  delay(500);
  
  Serial.begin(115200);
  Serial.println("Start chadding");

  // Initialise the ADB subsystem.  
  ADB::init();
  // If I do two-way comm on a single port I get some delays receiving data
  // every time after I send data
  // Splitting it out to one port dedictated to each direction seems to get 
  // around this.
  connectionSend = ADB::addConnection("tcp:45670", true, adbEventHandler);
  
  phoneIsConnected = false;
  lastPhoneUpdateTimeMS = millis();

}  

void loop() 
{
//  if((millis() - lastPhoneUpdateTimeMS) > 100)
//  {
//    if(phoneIsConnected > 0)
//      Serial.println("Lost the phone");
//    phoneIsConnected = false;
//  }
//  else if(phoneIsConnected && millis()-lastPhoneUpdateTimeMS > 30)
//  {
//    Serial.print("Long comm delay: ");
//    Serial.println(millis()-lastPhoneUpdateTimeMS);
//  }
  
  while(ADB::poll());

  delay(1);
} 





