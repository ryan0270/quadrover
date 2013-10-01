#include <Usb.h>
#include <adk.h>

USB Usb;
ADK adk(&Usb,"arduino", // Manufacturer Name
             "QuadRover", // Model Name
             "", // Description (user-visible string)
             "1.0", // Version
             "", // URL (web page to visit if no installed apps support the accessory)
             "123456789"); // Serial Number (optional)

#define LED 13 // Pin 13 is occupied by the SCK pin on a normal Arduino (Uno, Duemilanove etc.), so use a different pin

void setup()
{
  Serial.begin(115200);
  Serial.println("ADK demo start");
  if (Usb.Init() == -1) {
    Serial.println("OSCOKIRQ failed to assert");
    while(1); //halt
  }
  pinMode(LED, OUTPUT);
}

uint16_t vals[4];
void loop()
{    
  Usb.Task();
  if(adk.isReady()) 
  {
    byte msg[8];
    uint16_t len = 8;
    uint8_t rcode = adk.RcvData(&len, msg);
    if(len == 8)
    {
      for(int i=0; i<4; i++)
        vals[i] = msg[2*i] | (msg[2*i+1] << 8);
      printVals();
    }
    else if(len > 0)
    {
      Serial.print("Strange length: ");
      Serial.println(len);
    }
  } 
  else
    digitalWrite(LED, LOW); 
}

void printVals()
{
  Serial.print("vals: ");
  for(int i=0; i<4; i++)
  {
    Serial.print(vals[i]);
    Serial.print('\t');
  }
  Serial.print('\n');  
}
