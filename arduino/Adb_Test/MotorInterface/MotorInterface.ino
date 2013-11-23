#include <SPI.h>
#include <Adb.h>

boolean phoneIsConnectedReceive, phoneIsConnectedSend;
unsigned long lastPhoneUpdateTimeMS;

uint16_t motorCommands[4];

// Adb connection.
Connection *connectionReceive, *connectionSend;
// Event handler for the shell connection. 
void adbEventHandler(Connection * connection, adb_eventType event, uint16_t length, uint8_t * data)
{
  if(event == ADB_CONNECT)
  {    Serial.println("ADB_CONNECT");  }
  else if(event == ADB_DISCONNECT)
  {    phoneIsConnectedReceive = false;    Serial.println("ADB_DISCONNECT");  }
  else if(event == ADB_CONNECTION_OPEN)
  {    Serial.println("ADB_CONNECTION_OPEN receive");  }
  else if(event == ADB_CONNECTION_CLOSE)
  {    phoneIsConnectedReceive = false;    Serial.println("ADB_CONNECTION_CLOSE");
  }
  else if(event == ADB_CONNECTION_FAILED)
  {    phoneIsConnectedReceive = false;    Serial.println("ADB_CONNECTION_FAILED");
  }
  else if (event == ADB_CONNECTION_RECEIVE)
  {
    if(connection == connectionReceive)
    {
      if(!phoneIsConnectedReceive)
        Serial.println("Phone connected receive");
      phoneIsConnectedReceive = true;
      for(int i=0; i<4; i++)
      {
        short val = (data[2*i+1] << 8) | (data[2*i]);
        motorCommands[i] = val;
      }
  
      lastPhoneUpdateTimeMS = millis();
    }
    else
      Serial.println("Why is the send connection receiving data?");
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Start chadding");

  delay(500);
  // Initialise the ADB subsystem.  
  ADB::init();
  // If I do two-way comm on a single port I get some delays receiving data
  // every time after I send data
  // Splitting it out to one port dedictated to each direction seems to get 
  // around this.
  connectionReceive = ADB::addConnection("tcp:45670", true, adbEventHandler);
  connectionSend = ADB::addConnection("tcp:45671", true, adbEventHandler);

  phoneIsConnectedReceive = false;
  lastPhoneUpdateTimeMS = millis();
}  

enum
{
  COMM_ARDUINO_HEIGHT=1,
};

unsigned long lastHeightSendTimeMS = 0;
unsigned long lastMotorPrintTime = 0;
unsigned long loopStart = 0;
void loop() 
{
  loopStart = millis();
  if((millis() - lastPhoneUpdateTimeMS) > 100)
  {
    if(phoneIsConnectedReceive)
      Serial.println("Lost the phone");
    phoneIsConnectedReceive = false;
  }
//  else if((millis()-lastPhoneUpdateTimeMS) > 10)
//  {
//    Serial.print("Long comm wait: ");
//    Serial.println(millis()-lastPhoneUpdateTimeMS);
//  }

  if(millis()-lastHeightSendTimeMS > 50 && phoneIsConnectedSend)
  {
    uint16_t dt = millis()-lastHeightSendTimeMS;
    lastHeightSendTimeMS = millis();
    
    uint8_t code = COMM_ARDUINO_HEIGHT;
    uint8_t buff[3];
    buff[0] = code;
    memcpy(&(buff[1]),&dt,2);
    connectionSend->write(3,&(buff[0]));
  }
  
  if(millis()-lastMotorPrintTime > 1e3 && phoneIsConnectedReceive)
  {
    lastMotorPrintTime = millis();
    for(int i=0; i<4; i++)
    {
      Serial.print(motorCommands[i]);
      Serial.print("\t");
    }
    Serial.print("\n");
  }

  while(ADB::poll());
  
  unsigned long loopTime = millis()-loopStart;
  if(loopTime > 5)
  {
    Serial.print("Long loop time: ");
    Serial.println(loopTime);
  }
  delay(1);
} 

