#include <SPI.h>
#include <Wire.h>
#include <Adb.h>

#define SONAR_PIN 2

int verbosity=0;

enum
{
  COMM_ARDUINO_HEIGHT=1,
};

unsigned long lastSonarMeasureTimeMS = 0;
unsigned long sonarStartTime = 0;
boolean sonarIsHigh, sonarIsRunning = false;
boolean newSonarReady = false;
long sonarPulseLength = 0;

boolean phoneIsConnected;
unsigned long lastPhoneUpdateTimeMS;

byte MOTOR_ADDR_E = (0x53 + (0 << 1)) >> 1;
byte MOTOR_ADDR_W = (0x53 + (1 << 1)) >> 1;
byte MOTOR_ADDR_S = (0x53 + (2 << 1)) >> 1;
byte MOTOR_ADDR_N = (0x53 + (3 << 1)) >> 1;

byte motorAddr[4];
uint8_t sonarPort;
uint8_t sonarBit;
volatile uint8_t *sonarReg;
volatile uint8_t *sonarOut;

short motorCommands[4];

// Adb connection.
Connection *connectionReceive, *connectionSend;
// Event handler for the shell connection. 
void adbEventHandler(Connection *connection, adb_eventType event, uint16_t length, uint8_t *data)
{
  if(event == ADB_CONNECT)
  {    
    if(verbosity > 0)
      Serial.println("ADB_CONNECT");  
  }
  else if(event == ADB_DISCONNECT)
  {    
    phoneIsConnected = false;
    if(verbosity > 0)
      Serial.println("ADB_DISCONNECT");  
  }
  else if(event == ADB_CONNECTION_OPEN)
  {    
    if(verbosity > 0)
      Serial.println("ADB_CONNECTION_OPEN");  
      
    phoneIsConnected = true;
    lastPhoneUpdateTimeMS = millis();
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
    if(connection == connectionReceive)
    {
      phoneIsConnected = true;
      for(int i=0; i<4; i++)
      {
        short val = (data[2*i+1] << 8) | (data[2*i]);
        motorCommands[i] = val;
      }
  
      lastPhoneUpdateTimeMS = millis();
    }
    else if(verbosity > 0)
      Serial.println("Why is the send connection receiving data?");
  }
}

void setup()
{
  delay(500);
  
  if(verbosity > 0)
  {
    Serial.begin(115200);
    Serial.println("Start chadding");
  }

  // Initialise the ADB subsystem.  
  ADB::init();
  // If I do two-way comm on a single port I get some delays receiving data
  // every time after I send data
  // Splitting it out to one port dedictated to each direction seems to get 
  // around this.
  connectionReceive = ADB::addConnection("tcp:45670", true, adbEventHandler);
  connectionSend = ADB::addConnection("tcp:45671", true, adbEventHandler);
  
  Wire.begin();
  motorAddr[0] = MOTOR_ADDR_N;
  motorAddr[1] = MOTOR_ADDR_E;
  motorAddr[2] = MOTOR_ADDR_S;
  motorAddr[3] = MOTOR_ADDR_W;

  for(int i=0; i<4; i++)
  {
    motorCommands[i] = 0;
    sendMotorCommand(motorAddr[i], 0);
  }

  doRegularMotorStart();

  phoneIsConnected = false;
  lastPhoneUpdateTimeMS = millis();

  sonarPort = digitalPinToPort(SONAR_PIN);
  sonarBit = digitalPinToBitMask(SONAR_PIN);
  sonarReg = portModeRegister(sonarPort);
  sonarOut = portOutputRegister(sonarPort);

  attachInterrupt(0, handleSonarDone, CHANGE);
}  

void loop() 
{
  if((millis() - lastPhoneUpdateTimeMS) > 100)
  {
    if(phoneIsConnected && verbosity > 0)
      Serial.println("Lost the phone");
    phoneIsConnected = false;
  }
  else if(verbosity > 0 && phoneIsConnected && millis()-lastPhoneUpdateTimeMS > 30)
  {
    Serial.print("Long comm delay: ");
    Serial.println(millis()-lastPhoneUpdateTimeMS);
  }
  
  while(ADB::poll());

  // Send commands to the motors
  for(int i=0; i<4; i++)
  {
    if(!phoneIsConnected)
      motorCommands[i] = 0;

    sendMotorCommand(motorAddr[i], motorCommands[i]);
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

  // See if we've gotten the sonar echo back
  if(newSonarReady)
  {
    uint16_t height = sonarPulseLength/5.8;
    newSonarReady = false;
    sonarIsHigh = false;

    if(phoneIsConnected)
      sendVal(height);
  }

  if(millis()-lastSonarMeasureTimeMS > 50)
  {
    lastSonarMeasureTimeMS = millis();

    long range;
    // I'm using the direct c calls since they
    // are faster than the arduino versions
    *sonarReg |= sonarBit; // pinMode(SONAR_PIN,OUTPUT);
    *sonarOut &= ~sonarBit; // digitalWrite(SONAR_PIN,LOW);
    delayMicroseconds(2);
    *sonarOut |= sonarBit; // digitalWrite(SONAR_PIN,HIGH);
    delayMicroseconds(10);
    *sonarOut &= ~sonarBit; // digitalWrite(SONAR_PIN,LOW);

    sonarIsHigh = false;
    sonarIsRunning = true;    
    *sonarReg &= ~sonarBit; // pinMode(SONAR_PIN,INPUT)
    *sonarOut &= ~sonarBit;
    // The sonar echo event will be recorded by the interrupt handler
  }

  delayMicroseconds(500);
} 

void doRegularMotorStart()
{
  if(verbosity > 0)
    Serial.print("Starting motors in operation mode...");
  for(int i=0; i<100; i++)
  {
    // need to keep the comm alive
    for(int mdl=0; mdl<4; mdl++)
      sendMotorCommand(motorAddr[mdl], 0);
    delay(10);
  }
  if(verbosity > 0)
    Serial.println(" done");
}

void handleSonarDone()
{
  if(!sonarIsRunning)
    return;

  sonarIsHigh = !sonarIsHigh;
  if( sonarIsHigh )
    sonarStartTime = micros();
  else
  {
    sonarPulseLength = micros()-sonarStartTime;
    newSonarReady = true;
    sonarIsRunning = false;
  }
}

int sendVal(uint16_t val)
{
  uint8_t code = COMM_ARDUINO_HEIGHT;
  uint8_t buff[3];
  buff[0] = code;
  memcpy(&(buff[1]),&val,2);
  connectionSend->write(3,&(buff[0]));
  
  return 0;
}

boolean sendMotorCommand(byte addr, short cmd)
{
  Wire.beginTransmission(addr);
  byte upper = floor(cmd/8.0);
  byte lower = cmd % 8;
  Wire.write(upper);
  Wire.write( lower & 0x07 );
  byte result = Wire.endTransmission(true);
  return result == 0;
}




