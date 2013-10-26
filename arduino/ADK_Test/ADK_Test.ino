#include <Usb.h>
#include <adk.h>

USB Usb;
ADK adk(&Usb,"arduino", // Manufacturer Name
"QuadRover", // Model Name
"", // Description (user-visible string)
"1.0", // Version
"", // URL (web page to visit if no installed apps support the accessory)
"123456789"); // Serial Number (optional)

void setup()
{
  Serial.begin(115200);
  Serial.println("ADK demo start");
  if (Usb.Init() == -1) {
    Serial.println("OSCOKIRQ failed to assert");
    while(1); //halt
  }
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  digitalWrite(5,LOW);
  digitalWrite(6,LOW);
}

uint16_t vals[4];
unsigned long lastCommTime = 0;
unsigned long lastSendTime = 0;
boolean isConnected = false;
unsigned long startTime = 0;
void loop()
{    
  if(millis()-lastCommTime > 100)
  {
    if(!isConnected)
      digitalWrite(6,LOW);
    isConnected = false;
  }
  
  Usb.Task();
  if(adk.isReady()) 
  {
    if(startTime == 0)
      startTime = millis();
    boolean haveNewVals = false;
    while(readVals(vals) == 0)
    {
      if(!isConnected)
        digitalWrite(6,HIGH);
      isConnected = true;
      haveNewVals = true;
      if(millis()-lastCommTime > 10)
      {
        Serial.print("long comm time: ");
        Serial.println(millis()-lastCommTime);
      }
      lastCommTime = millis();
    }
    
//    if(haveNewVals)
//      printVals();
     
     if(millis()-lastSendTime > 50 && isConnected)
     {
       lastSendTime = millis();
       uint16_t sum = 0;
       for(int i=0; i<4; i++)
         sum += vals[i];
       
       sendVal(sum);
     }
  } 
    
  delay(1);
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
    Serial.println("Recovery");
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

void printVals()
{
  Serial.print("vals:\t");
  for(int i=0; i<4; i++)
  {
    Serial.print(vals[i]);
    Serial.print('\t');
  }
  Serial.print('\n');  
}

