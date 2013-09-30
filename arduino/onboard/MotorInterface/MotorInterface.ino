#include <SPI.h>
#include <Wire.h>
#include <Adb.h>

#define SONAR_PIN 2

int verbosity=1;

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
      //      byte val = data[i];
      short val = (data[2*i+1] << 8) | (data[2*i]);
      motorCommands[i] = val;
    }

    lastPhoneUpdateTimeMS = millis();
  }
}

boolean sendCommand(byte addr, short cmd)
{
//  Wire.beginTransmission(addr);
//  byte upper = floor(cmd/8.0);
//  byte lower = cmd % 8;
//  Wire.write(upper);
//  Wire.write( lower & 0x07 );
//  byte result = Wire.endTransmission(true);
//  return result == 0;
}

byte MOTOR_ADDR_E = (0x53 + (0 << 1)) >> 1;
byte MOTOR_ADDR_W = (0x53 + (1 << 1)) >> 1;
byte MOTOR_ADDR_S = (0x53 + (2 << 1)) >> 1;
byte MOTOR_ADDR_N = (0x53 + (3 << 1)) >> 1;


byte motorAddr[4];
uint8_t sonarPort;
uint8_t sonarBit;
volatile uint8_t *sonarReg;
volatile uint8_t *sonarOut;
void setup()
{
  if(verbosity > 0)
  {
    Serial.begin(57600);
    Serial.println("Start chadding");
  }

  delay(500);
Serial.println("1");
  // Initialise the ADB subsystem.  
  ADB::init();
  // Open an ADB stream to the phone's shell. Auto-reconnect
  connection = ADB::addConnection("tcp:45670", true, adbEventHandler);  
Serial.println("2");
  delay(1000);

//  Wire.begin();
  motorAddr[0] = MOTOR_ADDR_N;
  motorAddr[1] = MOTOR_ADDR_E;
  motorAddr[2] = MOTOR_ADDR_S;
  motorAddr[3] = MOTOR_ADDR_W;

  for(int i=0; i<4; i++)
  {
    motorCommands[i] = 0;
    sendCommand(motorAddr[i], 0);
  }

//  doRegularMotorStart();

  phoneIsConnected = false;
  lastPhoneUpdateTimeMS = millis();

  sonarPort = digitalPinToPort(SONAR_PIN);
  sonarBit = digitalPinToBitMask(SONAR_PIN);
  sonarReg = portModeRegister(sonarPort);
  sonarOut = portOutputRegister(sonarPort);

  attachInterrupt(0, handleSonarDone, CHANGE);
}  

enum
{
  COMM_ARDUINO_HEIGHT=1,
};

unsigned long lastHeightSendTimeMS = 0;
unsigned long sonarStartTime = 0;
boolean sonarIsHigh, sonarIsRunning = false;
;
boolean newSonarReady = false;
long sonarPulseLength = 0;
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

  if(newSonarReady)
  {
    uint16_t height = sonarPulseLength/5.8;
    newSonarReady = false;
    sonarIsHigh = false;

    if(phoneIsConnected)
    {
      uint8_t code = COMM_ARDUINO_HEIGHT;
      // ADB comm seems to wait for an ok reply which is needed
      // before it will send again. I don't want to wait for that
      // So I'll build everything into a single send
      uint8_t buff[3];
      buff[0] = code;
      memcpy(&(buff[1]),&height,2);
      connection->write(3,&(buff[0]));
    }
  }

  if(millis()-lastHeightSendTimeMS > 1000)
  {
    lastHeightSendTimeMS = millis();

    long range;
    // I'm using the direct c calls since they
    // are faster
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

  while(ADB::poll());
  
  delay(1);
} 

void doRegularMotorStart()
{
  if(verbosity > 0)
    Serial.print("Starting motors in operation mode...");
  for(int i=0; i<500; i++)
  {
    // need to keep the comm alive
    for(int mdl=0; mdl<4; mdl++)
      sendCommand(motorAddr[mdl], 0);
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



