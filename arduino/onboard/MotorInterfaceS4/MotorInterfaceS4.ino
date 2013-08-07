
union
{
  char byteVal[4];
  float floatVal;
  int intVal;
} 
serialFloat;

#define LED_PIN 12
unsigned long lastCommTime;
void setup()
{
  delay(500);
  Serial.begin(115200);
  pinMode(LED_PIN,OUTPUT);
  digitalWrite(LED_PIN,LOW);

  serialFloat.intVal = 0;
  while(Serial.available())
    Serial.read();

  lastCommTime = 0;
}

boolean isConnected = false;

float val = -1.325;
//int val = 0;
int timeoutMS = 500;
void loop()
{
  Serial.println(val,4);

  while(Serial.available() == 0)
  {
    if(millis()-lastCommTime > 500)
      isConnected = false;
    if(!isConnected)
      digitalWrite(LED_PIN,LOW);    
    delay(10);
  }

  if(Serial.available() > 0 )
  {
    int start = millis();
    while(Serial.available() < 4 && millis()-start < timeoutMS)
      delay(5);

    if(Serial.available() >= 4)
    {
      lastCommTime = millis();
      isConnected = true;
      digitalWrite(LED_PIN,HIGH);
      for(int i=0; i<4; i++)
        serialFloat.byteVal[i] = Serial.read();

      val = serialFloat.floatVal+0.25;
    }

    while(Serial.available() > 0)
      Serial.read();
  }

  delay(400);
}

