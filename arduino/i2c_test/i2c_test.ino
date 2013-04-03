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
short power = 0;
boolean readData = false;
void loop()
{
  byte addrs[8];
  addrs[0] = MOTOR_ADDR_N;
  addrs[1] = MOTOR_ADDR_E;
  addrs[2] = MOTOR_ADDR_S;
  addrs[3] = MOTOR_ADDR_W;
//  for(byte motorID = 0; motorID < 4; motorID++)
  byte motorID = 3;
  {
    Wire.beginTransmission(addrs[motorID]);
    byte upper = floor(power/8);
    byte lower = power % 8;
    Wire.write(upper);
    Wire.write( (1<<3) | (lower & 0x07));
    byte result = Wire.endTransmission(true);
    if(result != 0)
    {
      Serial.print('xmit error: ');
      Serial.println(result);
    }
  }
  Serial.print("\n");
  
  // rpm data seems to be in byte 3
  // byte 5 seems to have something, but not sure what yet (maybe voltage)
  // I'm not sure what the other bytes are
  Wire.requestFrom((int)addrs[motorID],5);
  delay(5);
  int n = Wire.available();
  Serial.print("\t");
  if(n > 0)
  {
    while(Wire.available() > 0)
    {
      byte b = Wire.read();
      Serial.print(b);
      Serial.print(" ");
    }
  }
  else
    Serial.print("x");    
  Serial.print("\n");

  if(millis() - startTime > 200)
  {
    power += 1;
    if(power > (5*8)-1)
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

  delay(50);
}


