#define LED_PIN 12
#define COMM_STATE_NONE_PIN 8
#define COMM_STATE_START_PIN 6
#define COMM_STATE_MOTOR_PIN 4

union
{
  char byteVal[4];
  float floatVal;
  int intVal;
} 
commMsg;

enum
{
  COMM_STATE_RECEIVED_START_MESSAGE,
  COMM_STATE_RECEIVED_MOTOR_ID,
  COMM_STATE_NONE,
};
int currentCommState;

enum
{
  COMM_FLAG_MESSAGE_PREFIX = 0xFFFF,
  COMM_FLAG_MESSAGE_SUFFIX = 0xFFFF-1,
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
int val = 10;
int timeoutMS = 500;
void loop()
{
//  if(isConnected)
//    analogWrite(LED_PIN, (int)(val*255));
//  else
//    digitalWrite(LED_PIN, LOW);

//  Serial.println(val,4);
  Serial.println(val);

  if(millis()-lastCommTime > 5000)
    isConnected = false;

//  if(Serial.available() > 0 )
//  {
//    int start = millis();
//    while(Serial.available() < 4 && millis()-start < timeoutMS)
//      delay(5);
//
//    if(Serial.available() >= 4)
//    {
//      lastCommTime = millis();
//      isConnected = true;
//      for(int i=0; i<4; i++)
//        commMsg.byteVal[i] = Serial.read();
//
////      if(commMsg.intVal == COMM_FLAG_MESSAGE_SUFFIX)
////      {
////        currentCommState = COMM_STATE_NONE;
////        digitalWrite(COMM_STATE_START_PIN,LOW);
////        digitalWrite(COMM_STATE_MOTOR_PIN,LOW);
////        digitalWrite(COMM_STATE_NONE_PIN,HIGH);
////      }
////      else
//      {  
//        switch(currentCommState)
//        {
//          case COMM_STATE_NONE:
////            val = commMsg.intVal;
//            if(commMsg.intVal == COMM_FLAG_MESSAGE_PREFIX)
//            {
//              currentCommState = COMM_STATE_RECEIVED_START_MESSAGE;
//              digitalWrite(COMM_STATE_NONE_PIN,LOW);
//              digitalWrite(COMM_STATE_START_PIN,HIGH);
//            }
//            break;
////          case COMM_STATE_RECEIVED_START_MESSAGE:
////            val = commMsg.floatVal;
////            currentCommState = COMM_STATE_NONE;
////            digitalWrite(COMM_STATE_START_PIN,LOW);
////            digitalWrite(COMM_STATE_MOTOR_PIN,LOW);
////            digitalWrite(COMM_STATE_NONE_PIN,HIGH);
////            break;
//          case COMM_STATE_RECEIVED_MOTOR_ID:
//            break;
//        }
//      }
//    }
//    else
//    {
//      currentCommState = COMM_STATE_NONE;
//        digitalWrite(COMM_STATE_START_PIN,LOW);
//        digitalWrite(COMM_STATE_MOTOR_PIN,LOW);
//        digitalWrite(COMM_STATE_NONE_PIN,HIGH);
//    }
//  }

  delay(100);
}


