
union
{
  char byteVal[4];
  float floatVal;
  int intVal;
} serialFloat;

void setup()
{
  delay(500);
  Serial.begin(115200);
  pinMode(13,OUTPUT);
  
  serialFloat.intVal = 0;
  while(Serial.available())
    Serial.read();
}

float val = -1.325;
//int val = 0;
int timeoutMS = 500;
void loop()
{
  Serial.println(val,4);
  
  while(Serial.available() == 0)
    delay(10);
    
  if(Serial.available() > 0 )
  {
    int start = millis();
    while(Serial.available() < 4 && millis()-start < timeoutMS)
      delay(5);
    
    if(Serial.available() >= 4)
    {
//      Serial.readBytes(serialFloat.byteVal, 4);
//      Serial.read();
//      serialFloat.intVal = Serial.parseInt();
//      Serial.readBytesUntil(0xFF, serialFloat.byteVal, 10);
      for(int i=0; i<4; i++)
        serialFloat.byteVal[i] = Serial.read();
      
      val = serialFloat.floatVal+0.25;
//      val = serialFloat.intVal+1;
//      val = Serial.parseInt()+1;
//      val = Serial.read()+1;
    }
    
    while(Serial.available() > 0)
      Serial.read();
  }
  
  delay(500);
}
