
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
  
  pinMode(9,OUTPUT);
  digitalWrite(9,LOW);
  pinMode(8,OUTPUT);
  digitalWrite(8,HIGH);

  serialFloat.intVal = 0;
  while(Serial.available())
    Serial.read();

  lastCommTime = 0;
}

boolean isConnected = false;

float val = 127/255;
//int val = 0;
int timeoutMS = 500;
void loop()
{
  val += 0.01;
  if(isConnected)
    analogWrite(LED_PIN, (int)(val*255));
  else
    digitalWrite(LED_PIN, LOW);
  
  Serial.println(val,4);

  if(millis()-lastCommTime > 500)
    isConnected = false;
  
  if(Serial.available() > 0 )
  {
    int start = millis();
    while(Serial.available() < 4 && millis()-start < timeoutMS)
      delay(5);

    if(Serial.available() >= 4)
    {
      lastCommTime = millis();
      isConnected = true;
//      digitalWrite(LED_PIN,HIGH);
      for(int i=0; i<4; i++)
        serialFloat.byteVal[i] = Serial.read();
        
      val = serialFloat.floatVal;
    }

    while(Serial.available() > 0)
      Serial.read();
  }

  delay(100);
}

