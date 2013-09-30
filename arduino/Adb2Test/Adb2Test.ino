#include <Usb.h>
#include <Adb2.h>

USB Usb;
Adb2::Adb2 adb(&Usb);

void setup()
{
  delay(500);
  Serial.begin(115200);
  Serial.println("Start chadding");
  if (Usb.Init() == -1) {
    Serial.println("\r\nOSCOKIRQ failed to assert");
    while(1); //halt
  }
}

void loop()
{    
  Usb.Task();
  delay(100);
}

