#define PIN_LED 12
#define PIN_MOTOR_0 8
#define PIN_MOTOR_1 6
#define PIN_MOTOR_2 4
#define PIN_MOTOR_3 2
#define NUM_MOTORS 4
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

unsigned long lastCommTime;
boolean isConnected = false;
int timeoutMS = 500;
int motorVals[NUM_MOTORS];
int motorPins[4];
void setup()
{
  delay(500);
  Serial.begin(115200);
  pinMode(PIN_LED,OUTPUT);
  digitalWrite(PIN_LED,LOW);

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
  pinMode(PIN_MOTOR_0,OUTPUT); 
  digitalWrite(PIN_MOTOR_0,LOW);
  pinMode(PIN_MOTOR_1,OUTPUT); 
  digitalWrite(PIN_MOTOR_1,LOW);
  pinMode(PIN_MOTOR_2,OUTPUT); 
  digitalWrite(PIN_MOTOR_2,LOW);
  pinMode(PIN_MOTOR_3,OUTPUT); 
  digitalWrite(PIN_MOTOR_3,LOW);

  currentCommState = COMM_STATE_NONE;

  commMsg.intVal = 0;
  while(Serial.available())
    Serial.read();

  lastCommTime = 0;

  motorVals[0] = motorVals[1] = motorVals[2] = motorVals[3] = 0;

  motorPins[0] = PIN_MOTOR_0;
  motorPins[1] = PIN_MOTOR_1;
  motorPins[2] = PIN_MOTOR_2;
  motorPins[3] = PIN_MOTOR_3;
}

byte nextMotorID = 0;
void loop()
{
  if(millis()-lastCommTime > 1000)
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

  while(Serial.available() > 0 )
  {
    int start = millis();
    
    while(Serial.available() < 4 && millis()-start < timeoutMS)
      delay(1);
      
    if(Serial.available() >= 4)
    {
      for(int b=0; b<4; b++)
        commMsg.byteVal[b] = Serial.read();
        
      if( commMsg.uint32Val == COMM_FLAG_MESSAGE_PREFIX )
      {
        lastCommTime = millis();
        if(!isConnected)
          digitalWrite(PIN_LED, HIGH);
        isConnected = true;
        currentCommState = COMM_STATE_RECEIVED_START_MESSAGE;
      }
      else if(currentCommState == COMM_STATE_RECEIVED_START_MESSAGE)
      {
        motorVals[nextMotorID] = (commMsg.uint32Val & 0x0000FFFF); // ints on the mega ADK are 16-bit
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






