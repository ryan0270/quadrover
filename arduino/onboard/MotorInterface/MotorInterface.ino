// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.

// it look like values between 30 and ~50 are ok to start the motor in normal
// operation. At 49 the motor starts to actually spin. Values below 30
// seem to startup in some form of configuration mode

#include <SPI.h>
//#include <Adb.h>
#include <Wire.h>

int verbosity=0;

byte motorCommands[4];

boolean phoneIsConnected;
unsigned long lastPhoneUpdateTimeMS;

//// Adb connection.
//Connection * connection;
//// Event handler for the shell connection. 
//void adbEventHandler(Connection * connection, adb_eventType event, uint16_t length, uint8_t * data)
//{
//  if(event == ADB_CONNECT && verbosity > 0)
//  {
//    Serial.println("ADB_CONNECT");
//  }
//  else if(event == ADB_DISCONNECT)
//  {
//    phoneIsConnected = false;
//    if(verbosity > 0)
//      Serial.println("ADB_DISCONNECT");
//  }
//  else if(event == ADB_CONNECTION_OPEN && verbosity > 0)
//  {
//    Serial.println("ADB_CONNECTION_OPEN");
//  }
//  else if(event == ADB_CONNECTION_CLOSE)
//  {
//    phoneIsConnected = false;
//    if(verbosity > 0)
//      Serial.println("ADB_CONNECTION_CLOSE");
//  }
//  else if(event == ADB_CONNECTION_FAILED)
//  {
//    phoneIsConnected = false;
//    if(verbosity > 0)
//      Serial.println("ADB_CONNECTION_FAILED");
//  }
//  else if (event == ADB_CONNECTION_RECEIVE)
//  {
//    phoneIsConnected = true;
//    for(int i=0; i<4; i++)
//    {
//      byte val = data[2*i];
//      motorCommands[i] = val;
//    }
//    
//    lastPhoneUpdateTimeMS = millis();
//  }
//}

boolean sendCommand(byte addr, byte cmd)
{
  Wire.beginTransmission(addr);
  Wire.write(cmd);
  byte result = Wire.endTransmission(true);
  return result == 0;  
}
 
byte motorAddr[4];
void setup()
{ 
  motorAddr[0] = (0x53+(2<<1))>>1;
  motorAddr[1] = (0x53+(1<<1))>>1;
  motorAddr[2] = (0x53+(0<<1))>>1;
  motorAddr[3] = (0x53+(3<<1))>>1;
    
  for(int i=0; i<4; i++)
  {
    motorCommands[i] = 0;
    sendCommand(motorAddr[i], 0);
  }
  
  if(verbosity > 0)
  {
    Serial.begin(57600);
    Serial.println("Start chadding");
  }
  delay(1000);
  
  // Initialise the ADB subsystem.  
  ADB::init();
  // Open an ADB stream to the phone's shell. Auto-reconnect
  connection = ADB::addConnection("tcp:45670", true, adbEventHandler);  

  doRegularMotorStart();

  phoneIsConnected = false;
  lastPhoneUpdateTimeMS = millis();
}  
 
byte power=0;
unsigned long startTim
void loop() 
{    
  for(byte motorID = 0; motorID < 4; motorID++)
  {
//    byte writeAddr = MOTOR_BASE_ADDR+(motorID << 1);
//    Wire.beginTransmission(writeAddr >> 1);
    Wire.beginTransmission(motorAddr[motorID]);
    Wire.write(power);
    byte result = Wire.endTransmission(true);
    if(result != 0)
    {
      Serial.print('xmit error: ');
      Serial.println(result);
    }
  }
  
  if(millis() - startTime > 1000)
  {
    power += 1;
    if(power == 1) power = 3; // power of 1 is crappy
    else if(power > 10)
      power = 0;
    startTime = millis();
    Serial.print("power: ");
    Serial.println(power);
  }
    
    ADB::poll();
    delay(10);
  return;
  if((millis() - lastPhoneUpdateTimeMS) > 100)
    phoneIsConnected = false;  
    
  for(int i=0; i<4; i++)
  {
    if(!phoneIsConnected)
      motorCommands[i] = 0;
      
    sendCommand(motorAddr[i], motorCommands[i]);
    if(verbosity >=2 )
    {
      Serial.print(motorCommands[i]);
      Serial.print("\t");
    }
  }
  if(verbosity >= 2)
  {
    if(phoneIsConnected)
      Serial.print("***");
    Serial.print("\n");    
  }
  ADB::poll();
  delay(5);
} 

void doRegularMotorStart()
{
  if(verbosity > 0)
    Serial.print("Starting motors in operation mode...");
  for(int i=0; i<500; i++)
  {
    // need to keep the comm alive
    for(int mdl=0; mdl<4; i++)
      sendCommand(motorAddr[i], 0);
    delay(10);
  }
  if(verbosity > 0)
    Serial.println(" done");
}

