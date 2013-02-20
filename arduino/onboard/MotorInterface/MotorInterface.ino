// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.

// it look like values between 30 and ~50 are ok to start the motor in normal
// operation. At 49 the motor starts to actually spin. Values below 30
// seem to startup in some form of configuration mode

#include <SPI.h>
#include <Adb.h>
#include <Servo.h>  // header file needs to be updated to use a 5000us refresh rate


int verbosity=0;

//Servo servoN, servoE, servoS, servoW;
Servo servos[4];
short motorCommands[4];

boolean phoneIsConnected;
unsigned long lastPhoneUpdateTimeMS;

// Adb connection.
Connection * connection;
// Event handler for the shell connection. 
void adbEventHandler(Connection * connection, adb_eventType event, uint16_t length, uint8_t * data)
{
  if(event == ADB_CONNECT && verbosity > 0)
  {
    Serial.println("ADB_CONNECT");
  }
  else if(event == ADB_DISCONNECT)
  {
    phoneIsConnected = false;
    if(verbosity > 0)
      Serial.println("ADB_DISCONNECT");
  }
  else if(event == ADB_CONNECTION_OPEN && verbosity > 0)
  {
    Serial.println("ADB_CONNECTION_OPEN");
  }
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
  servos[0].attach(5);
  servos[1].attach(7);
  servos[2].attach(3); // I don't know why, but pin 9 wasn't working. Maybe it has a conflict with ADB
  servos[3].attach(11);
  for(int i=0; i<4; i++)
    servos[i].writeMicroseconds(1000);
    
  for(int i=0; i<4; i++)
    motorCommands[i] = 1000;
  
  if(verbosity > 0)
  {
    Serial.begin(57600);
    Serial.println("Start chadding");
  }
  delay(1000);
  
  // Initialise the ADB subsystem.  
  ADB::init();
  // Open an ADB stream to the phone's shell. Auto-reconnect
  connection = ADB::addConnection("tcp:4567", true, adbEventHandler);  
  
  doRegularMotorStart();
//  doThrottleRangeSet(); Serial.begin(57600);

  phoneIsConnected = false;
  lastPhoneUpdateTimeMS = millis();
}  
 
void loop() 
{    
  if((millis() - lastPhoneUpdateTimeMS) > 100)
    phoneIsConnected = false;  
    
  for(int i=0; i<4; i++)
  {
    if(!phoneIsConnected)
      motorCommands[i] = 1000;
      
    servos[i].writeMicroseconds((int)motorCommands[i]);
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
  for(int i=0; i<4; i++)
    servos[i].write(30);
  delay(5000);
  if(verbosity > 0)
    Serial.println(" done");
}

void doThrottleRangeSet()
{
  Serial.println("--- Throttle range setting mode ---");
  Serial.println("--- Make sure the battery is disconnected when starting this mode. ---\n");
  delay(5000);
  Serial.print("Setting throttle to high ...");
  for(int i=0; i<4; i++)
    servos[i].writeMicroseconds(2000);
  Serial.println("done");
  Serial.print("Prepare to connect battery ... ");
  Serial.print("5 ... ");
  delay(1000); Serial.print("4 ... ");
  delay(1000); Serial.print("3 ... ");
  delay(1000); Serial.print("2 ... ");
  delay(1000); Serial.print("1 ... ");
  delay(1000); Serial.println("0");
  Serial.println("Connect battery. \nAfter about 2 seconds you should hear BEEP-BEEP indicating that the high point has been set.");
  Serial.print("5 ... ");
  delay(1000); Serial.print("4 ... ");
  delay(1000); Serial.print("3 ... ");
  delay(1000); Serial.print("2 ... ");
  delay(1000); Serial.print("1 ... ");
  delay(1000); Serial.println("0");
  Serial.print("Setting throttle to low ...");
  for(int i=0; i<4; i++)
    servos[i].writeMicroseconds(1000);
  Serial.println("done\n");
  Serial.println("You should soon hear a BEEP for every battery cell present and then a long beep confirming that the low point has been set.");
  delay(5000);

  Serial.println("/// Throttle range setting done \\\\\\");
}
