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
  Serial.print("\r\nADK demo start");
  if (Usb.Init() == -1) {
    Serial.print("\r\nOSCOKIRQ failed to assert");
    while(1); //halt
  }
  pinMode(LED, OUTPUT);
}

void loop()
{    
  Usb.Task();
  if(adk.isReady()) {
    uint8_t msg[1];
    uint16_t len = sizeof(msg);
    uint8_t rcode = adk.RcvData(&len, msg);
    if(len > 0) {
      Serial.print("Data Packet: ");
      Serial.println(msg[0]);
      digitalWrite(LED,msg[0] ? HIGH : LOW);
    }
  } 
  else
    digitalWrite(LED, LOW); 
}
