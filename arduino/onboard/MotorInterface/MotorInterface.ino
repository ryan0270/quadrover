#include <SPI.h>
#include <Wire.h>
#include <USB.h>
#include <adk.h>

#define SONAR_PIN 2

int verbosity=1;

USB Usb;
ADK adk(&Usb,"arduino", // Manufacturer Name
"QuadRover", // Model Name
"", // Description (user-visible string)
"1.0", // Version
"", // URL (web page to visit if no installed apps support the accessory)
"123456789"); // Serial Number (optional)

short motorCommands[4];

boolean phoneIsConnected;
unsigned long lastPhoneUpdateTimeMS;

boolean sendCommand(byte addr, short cmd)
{
  Wire.beginTransmission(addr);
  byte upper = floor(cmd/8.0);
  byte lower = cmd % 8;
  Wire.write(upper);
  Wire.write( lower & 0x07 );
  byte result = Wire.endTransmission(true);
  return result == 0;
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
  delay(500);
  
  if(verbosity > 0)
  {
    Serial.begin(115200);
    Serial.println("Start chadding");
  }

  if (Usb.Init() == -1)
    while(1); //halt
  
  Wire.begin();
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

unsigned long lastSonarMeasureTimeMS = 0;
unsigned long sonarStartTime = 0;
boolean sonarIsHigh, sonarIsRunning = false;
boolean newSonarReady = false;
long sonarPulseLength = 0;
void loop() 
{
  if((millis() - lastPhoneUpdateTimeMS) > 100)
  {
    if(phoneIsConnected && verbosity >= 1)
      Serial.println("Lost the phone");
    phoneIsConnected = false;
  }

  // this triggers the poll on the usb interface
  Usb.Task();
  if(adk.isReady()) 
  {
    boolean haveNewVals = false;
    uint16_t vals[4];
    while(readVals(vals) == 0)
    {
      if(verbosity > 0 && millis()-lastPhoneUpdateTimeMS > 10)
      {
        Serial.print("Long comm delay: ");
        Serial.println(millis()-lastPhoneUpdateTimeMS);
      }
      phoneIsConnected = true;
      haveNewVals = true;
      lastPhoneUpdateTimeMS = millis();
    }

    if(haveNewVals)
      for(int i=0; i<4; i++)
        motorCommands[i] = vals[i];
  } 

  // Send commands to the motors
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

  // See if we've gotten the sonar echo back
  if(newSonarReady)
  {
    uint16_t height = sonarPulseLength/5.8;
    newSonarReady = false;
    sonarIsHigh = false;

    if(phoneIsConnected)
    {
      //      uint8_t code = COMM_ARDUINO_HEIGHT;
      //      uint8_t buff[3];
      //      buff[0] = code;
      //      memcpy(&(buff[1]),&height,2);
      //      connection->write(3,&(buff[0]));
      sendVal(height);
    }
  }

  if(millis()-lastSonarMeasureTimeMS > 50)
  {
    lastSonarMeasureTimeMS = millis();

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

int readVals(uint16_t vals[])
{
  byte msg[8];
  uint16_t len = 8;
  int received = 0;
  uint8_t rcode = adk.RcvData(&len, msg);
  received = len;
  unsigned long start = millis();
  int timeoutMS = 100;
  while(rcode == 0 && received < 8 && (millis()-start) < timeoutMS)
  {
    len = 8-received;
    rcode = adk.RcvData(&len, &(msg[received-1]));
    received += len;
  }

  if(millis()-start > timeoutMS)
    return -10;

  if(rcode != 0)
    return rcode;

  for(int i=0; i<4; i++)
    vals[i] = (msg[2*i] << 8) | msg[2*i+1];

  return rcode;
}

int sendVal(uint16_t val)
{
  byte buff[2];
  buff[0] = (byte)(val >> 8); 
  buff[1] = (byte)val;
  uint8_t rcode = adk.SndData(2, buff);

  return rcode;
}




