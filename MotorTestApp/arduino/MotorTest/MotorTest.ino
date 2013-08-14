#include <Wire.h>

#define PIN_LED 12
#define PIN_MOTOR_0 8
#define PIN_MOTOR_1 6
#define PIN_MOTOR_2 4
#define PIN_MOTOR_3 2
#define NUM_MOTORS 4

byte MOTOR_ADDR_E = (0x53 + (0 << 1)) >> 1;
byte MOTOR_ADDR_W = (0x53 + (1 << 1)) >> 1;
byte MOTOR_ADDR_S = (0x53 + (2 << 1)) >> 1;
byte MOTOR_ADDR_N = (0x53 + (3 << 1)) >> 1;
byte motorAddr[4];

union
{
  byte byteVal[4];
  float floatVal;
  int intVal;
  unsigned int uintVal;
  short shortVal;
  uint32_t uint32Val;
} 
commMsg;

enum
{
  COMM_STATE_NONE,
  COMM_STATE_RECEIVED_START_MESSAGE,  
};
int currentCommState;

int MAXCMD = 1<<11;

enum
{
  COMM_FLAG_MESSAGE_PREFIX = 0x00CCBBAA,
};

unsigned long lastUsbCommTime, lastEscCommTime;
boolean isConnected = false;
int timeoutMS = 500;
int motorVals[NUM_MOTORS];
int motorPins[4];
void setup()
{
  delay(500);
  
  // Ground pins
  pinMode(9,OUTPUT); 
  digitalWrite(9,LOW);
  pinMode(7,OUTPUT); 
  digitalWrite(7,LOW);
  pinMode(5,OUTPUT); 
  digitalWrite(5,LOW);
  pinMode(3,OUTPUT); 
  digitalWrite(3,LOW);

  // active pins
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_MOTOR_0,OUTPUT); 
  digitalWrite(PIN_MOTOR_0,LOW);
  pinMode(PIN_MOTOR_1,OUTPUT); 
  digitalWrite(PIN_MOTOR_1,LOW);
  pinMode(PIN_MOTOR_2,OUTPUT); 
  digitalWrite(PIN_MOTOR_2,LOW);
  pinMode(PIN_MOTOR_3,OUTPUT); 
  digitalWrite(PIN_MOTOR_3,LOW);
  
  motorAddr[0] = MOTOR_ADDR_N;
  motorAddr[1] = MOTOR_ADDR_E;
  motorAddr[2] = MOTOR_ADDR_S;
  motorAddr[3] = MOTOR_ADDR_W;

  currentCommState = COMM_STATE_NONE;
  commMsg.intVal = 0;
  
  lastUsbCommTime = 0;
  lastEscCommTime = 0;

  motorPins[0] = PIN_MOTOR_0;
  motorPins[1] = PIN_MOTOR_1;
  motorPins[2] = PIN_MOTOR_2;
  motorPins[3] = PIN_MOTOR_3;
  
  Wire.begin();  
  for(int i=0; i<4; i++)
  {
    motorVals[i] = 0;
//    sendCommand(motorAddr[i], 0);
    Wire.beginTransmission(motorAddr[i]);
    byte upper = 0;
    byte lower = 0;
    Wire.write(upper);
    Wire.write( lower & 0x07 );
    byte result = Wire.endTransmission(true);
  }

  doRegularMotorStart();

  Serial.begin(115200);
  while(Serial.available())
    Serial.read(); 
}

byte nextMotorID = 0;
void loop()
{
  if(millis()-lastUsbCommTime > 1000)
  {
    if(isConnected)
    {
      digitalWrite(PIN_LED, LOW);
      for(int motorID=0; motorID<NUM_MOTORS; motorID++)
      {
        motorVals[motorID] = 0;
        digitalWrite(motorPins[motorID], LOW);
      }
    }
    isConnected = false;
  }
  
  // for safety and to maintain the comm link with
  // ESCs, this will happen every loop
  if(!isConnected && millis()-lastEscCommTime > 50)
  {
    lastEscCommTime = millis();
    for(int motorID=0; motorID<NUM_MOTORS; motorID++)
      sendCommand(motorAddr[motorID], 0);
  }

  if(Serial.available() > 0 )
  {
    int start = millis();
    
    while(Serial.available() < 4 && millis()-start < timeoutMS)
      delay(1);
      
    if(Serial.available() >= 4)
    {
      for(int b=0; b<4; b++)
        commMsg.byteVal[b] = Serial.read();
        
      if( commMsg.uint32Val == COMM_FLAG_MESSAGE_PREFIX)
//      if( (commMsg.uint32Val & 0xCCBBAA00) == 0xCCBBAA00 )
      {
        nextMotorID = 0;
        lastUsbCommTime = millis();
//        if(!isConnected)
          digitalWrite(PIN_LED, HIGH);
        isConnected = true;
        currentCommState = COMM_STATE_RECEIVED_START_MESSAGE;
      }
      else if(currentCommState == COMM_STATE_RECEIVED_START_MESSAGE)
      {
        motorVals[nextMotorID] = (commMsg.uint32Val & 0x0000FFFF); // ints on the mega ADK are 16-bit
//        sendCommand(motorAddr[nextMotorID], motorVals[nextMotorID]);
        analogWrite(motorPins[nextMotorID], (int)( (float)motorVals[nextMotorID]/MAXCMD*255));
        nextMotorID = (nextMotorID+1) % 4;
      }
    }
    else
    {
      while(Serial.available() > 0)
        Serial.read();
      for(int motorID=0; motorID<NUM_MOTORS; motorID++)
      {
        motorVals[motorID] = 0;
        analogWrite(motorPins[motorID], 0);
      }
      currentCommState = COMM_STATE_NONE;
    }
  }

  delay(1);
}

void doRegularMotorStart()
{
  for(int i=0; i<500; i++)
  {
    // need to keep the comm alive
    for(int mdl=0; mdl<4; mdl++)
      sendCommand(motorAddr[mdl], 0);
    delay(10);
  }
}

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




