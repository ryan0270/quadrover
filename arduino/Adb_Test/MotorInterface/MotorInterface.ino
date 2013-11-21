#include <SPI.h>
#include <Adb.h>

int verbosity=1;

boolean phoneIsConnected;
unsigned long lastPhoneUpdateTimeMS;

uint16_t motorCommands[4];

// Adb connection.
Connection * connection;
// Event handler for the shell connection. 
void adbEventHandler(Connection * connection, adb_eventType event, uint16_t length, uint8_t * data)
{
  if(event == ADB_CONNECT && verbosity > 0)
  {    Serial.println("ADB_CONNECT");  }
  else if(event == ADB_DISCONNECT)
  {
    phoneIsConnected = false;
    if(verbosity > 0)
      Serial.println("ADB_DISCONNECT");
  }
  else if(event == ADB_CONNECTION_OPEN && verbosity > 0)
  {    Serial.println("ADB_CONNECTION_OPEN");  }
  else if(event == ADB_CONNECTION_CLOSE)
  {
    phoneIsConnected = false;
    if(verbosity > 0)
      Serial.println("ADB_CONNECTION_CLOSE");
  }
  else if(event == ADB_CONNECTION_FAILED)
  {
    phoneIsConnected = false;
    if(verbosity > 0)
      Serial.println("ADB_CONNECTION_FAILED");
  }
  else if (event == ADB_CONNECTION_RECEIVE)
  {
    if(!phoneIsConnected && verbosity > 0)
      Serial.println("Phone connected");
    phoneIsConnected = true;
    for(int i=0; i<4; i++)
    {
      short val = (data[2*i+1] << 8) | (data[2*i]);
      motorCommands[i] = val;
    }

    lastPhoneUpdateTimeMS = millis();
  }
}

void setup()
{
  if(verbosity > 0)
  {
    Serial.begin(115200);
    Serial.println("Start chadding");
  }

  delay(500);
  // Initialise the ADB subsystem.  
  ADB::init();
  // Open an ADB stream to the phone's shell. Auto-reconnect
  connection = ADB::addConnection("tcp:45670", true, adbEventHandler);  

  phoneIsConnected = false;
  lastPhoneUpdateTimeMS = millis();
}  

enum
{
  COMM_ARDUINO_HEIGHT=1,
};

unsigned long lastHeightSendTimeMS = 0;
unsigned long loopStart = 0;
void loop() 
{
  if((millis() - lastPhoneUpdateTimeMS) > 100)
  {
    if(phoneIsConnected && verbosity >= 1)
      Serial.println("Lost the phone");
    phoneIsConnected = false;
  }
  else if((millis()-lastPhoneUpdateTimeMS) > 30)
    Serial.println(millis()-lastPhoneUpdateTimeMS);

  if(millis()-lastHeightSendTimeMS > 1000)
  {
    uint16_t dt = millis()-lastHeightSendTimeMS;
    lastHeightSendTimeMS = millis();
    
    if(phoneIsConnected)
    {
      uint8_t code = COMM_ARDUINO_HEIGHT;
      // ADB comm seems to wait for an ok reply which is needed
      // before it will send again. I don't want to wait for that
      // So I'll build everything into a single send
      uint8_t buff[3];
      buff[0] = code;
      memcpy(&(buff[1]),&dt,2);
      connection->write(3,&(buff[0]));
    }
  }

  while(ADB::poll());
  
  delay(1);
} 

