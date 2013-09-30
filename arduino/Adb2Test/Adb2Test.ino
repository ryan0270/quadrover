#include <Usb.h>
#include <Adb2.h>

USB Usb;
Adb2::Adb2 adb(&Usb);

#define LED 13 // Pin 13 is occupied by the SCK pin on a normal Arduino (Uno, Duemilanove etc.), so use a different pin

void setup()
{
  delay(500);
  Serial.begin(57600);
  Serial.println("Start chadding");
  if (Usb.Init() == -1) {
    Serial.println("\r\nOSCOKIRQ failed to assert");
    while(1); //halt
  }
  pinMode(LED, OUTPUT);
}

void loop()
{    
  Usb.Task();

  delay(10);
}

