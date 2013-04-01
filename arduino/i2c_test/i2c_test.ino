#include <Wire.h>

unsigned long startTime = 0;
void setup()
{
  delay(1000);
  
  Serial.begin(57600);   
  Serial.println("start chadding");
  
  Wire.begin(); // join i2c bus
  startTime = millis();
}

byte MOTOR_BASE_ADDR = 0x53;
byte MOTOR_ADDR_S = (0x53 + (0 << 1)) >> 1;
byte MOTOR_ADDR_N = (0x53 + (2 << 1)) >> 1;
byte MOTOR_ADDR_E = (0x53 + (1 << 1)) >> 1;
byte MOTOR_ADDR_W = (0x53 + (3 << 1)) >> 1;
int cnt = 0;
byte power = 0;
void loop()
{
  byte addrs[4];
  addrs[0] = MOTOR_ADDR_N;
  addrs[1] = MOTOR_ADDR_E;
  addrs[2] = MOTOR_ADDR_S;
  addrs[3] = MOTOR_ADDR_W;
  for(byte motorID = 1; motorID < 4; motorID++)
//  byte motorID = 3;
  {
//    byte writeAddr = MOTOR_BASE_ADDR+(motorID << 1);
//    Wire.beginTransmission(writeAddr >> 1);
    Wire.beginTransmission(addrs[motorID]);
    Wire.write(power);
    byte result = Wire.endTransmission(true);
    if(result != 0)
    {
      Serial.print('xmit error: ');
      Serial.println(result);
    }
  }

  if(millis() - startTime > 1000)
  {
    power += 1;
    if(power == 1) power = 3; // power of 1 is crappy
    else if(power > 10)
      power = 0;
    startTime = millis();
    Serial.print("power: ");
    Serial.println(power);
  }
//  Wire.requestFrom((int)(readAddr >> 1), 1);
//  delay(10);
//  if(Wire.available())
//  {
//    byte c = Wire.read(); // This should be the motor current
//    Serial.print(c,HEX);
//  }
//  else
//    Serial.print('F');  
//  if(++cnt > 60)
//  {
//    cnt = 0;
//    Serial.print('\n');
//  }
 
  delay(10);
}

