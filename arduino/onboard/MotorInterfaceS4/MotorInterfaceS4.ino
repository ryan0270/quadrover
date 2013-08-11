#define LED_PIN 12
#define COMM_STATE_NONE_PIN 8
#define COMM_STATE_START_PIN 6
#define COMM_STATE_MOTOR_PIN 4

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
  COMM_STATE_UNKNOWN=-1,
  COMM_STATE_RECEIVED_START_MESSAGE,
  COMM_STATE_RECEIVED_MOTOR_ID,
  COMM_STATE_NONE,
};
int currentCommState;

enum
{
  COMM_FLAG_MESSAGE_PREFIX = 0xDDCCBBAA,
  COMM_FLAG_MESSAGE_SUFFIX = 0xDDCCBBAA-2,
};

unsigned long lastCommTime;
void setup()
{
  delay(500);
  Serial.begin(115200);
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW);

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
  pinMode(COMM_STATE_NONE_PIN,OUTPUT); digitalWrite(COMM_STATE_NONE_PIN,HIGH);
  pinMode(COMM_STATE_START_PIN,OUTPUT); digitalWrite(COMM_STATE_START_PIN,LOW);
  pinMode(COMM_STATE_MOTOR_PIN,OUTPUT); digitalWrite(COMM_STATE_MOTOR_PIN,LOW);
  pinMode(2,OUTPUT); digitalWrite(2,LOW);
  
  currentCommState = COMM_STATE_NONE;

  commMsg.intVal = 0;
  while(Serial.available())
    Serial.read();

  lastCommTime = 0;
}

boolean isConnected = false;

//float val = 127/255;
float val = 10;
int timeoutMS = 500;
void loop()
{
  val += 0.01;
  
  if(isConnected)
    analogWrite(LED_PIN, (int)(val*255));
  else
    digitalWrite(LED_PIN, LOW);

  Serial.println(val,4);

  if(millis()-lastCommTime > 5000)
    isConnected = false;

  while(Serial.available() > 0 )
  {
    int start = millis();
    while(Serial.available() < 4 && millis()-start < timeoutMS)
      delay(5);

    if(Serial.available() >= 4)
    {
      lastCommTime = millis();
      isConnected = true;
      for(int bn=0; bn<4; bn++)
        commMsg.byteVal[bn] = Serial.read();
        
      if(commMsg.uint32Val == COMM_FLAG_MESSAGE_SUFFIX)
      {
        digitalWrite(4,LOW);
        currentCommState = COMM_STATE_NONE;
        digitalWrite(COMM_STATE_START_PIN,LOW);
        digitalWrite(COMM_STATE_MOTOR_PIN,LOW);
        digitalWrite(COMM_STATE_NONE_PIN,HIGH);
      }
      else
      {  
        switch(currentCommState)
        {
          case COMM_STATE_NONE:
            if( commMsg.uint32Val == COMM_FLAG_MESSAGE_PREFIX)
            {
              currentCommState = COMM_STATE_RECEIVED_START_MESSAGE;
//              digitalWrite(COMM_STATE_NONE_PIN,LOW);
//              digitalWrite(COMM_STATE_START_PIN,HIGH);
////              PORTB &= ~_BV(PB0); // Pin 8 low
////              PORTD |= _BV(PD6); // Pin 6 high
            }
            break;
          case COMM_STATE_RECEIVED_START_MESSAGE:
            val = commMsg.floatVal;
            currentCommState = COMM_STATE_UNKNOWN;
            digitalWrite(4,HIGH);
//            digitalWrite(COMM_STATE_START_PIN,LOW);
//            digitalWrite(COMM_STATE_MOTOR_PIN,LOW);
//            digitalWrite(COMM_STATE_NONE_PIN,HIGH);
////            PORTB |= _BV(PB0); // Pin 8 high
////            PORTD &= ~_BV(PD6); // Pin 6 low
            break;
          case COMM_STATE_RECEIVED_MOTOR_ID:
            break;
        }
      }
    }
    else
    {
      while(Serial.available() > 0)
        Serial.read();
      digitalWrite(2,HIGH);
      delay(100);
      digitalWrite(2,LOW);
      currentCommState = COMM_STATE_NONE;
      digitalWrite(COMM_STATE_START_PIN,LOW);
      digitalWrite(COMM_STATE_MOTOR_PIN,LOW);
      digitalWrite(COMM_STATE_NONE_PIN,HIGH);
    }
  }

  delay(100);
}


