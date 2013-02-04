// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.

// it look like values between 30 and ~50 are ok to start the motor in normal
// operation. At 49 the motor starts to actually spin. Values below 30
// seem to startup in some form of configuration mode

#include <Servo.h> 
 
Servo servoN, servoE, servoS, servoW;
 
int posN, posE, posS, posW;

 
void setup() 
{ 
  servoN.attach(5);
  servoE.attach(7);
  servoS.attach(3);
  servoW.attach(11);
  servoN.writeMicroseconds(1000);
  servoE.writeMicroseconds(1000);
  servoS.writeMicroseconds(1000);
  servoW.writeMicroseconds(1000);

//  pinMode(13,OUTPUT);
  posN = posE = posS = posW = 0;    // variable to store the servo position 
  
  Serial.begin(57600);
  Serial.println("Start chadding");
  delay(1000);
  
  doRegularMotorStart();
//  doThrottleRangeSet();
} 
 
 
void loop() 
{  
//  return;
  // below 800 looks like it's some sort of setup mode (slow constant beeping)
  // above 1050 starts to actually spin the motor
  // It seems like the motor still spins up in discrete steps. It may be that the 
  // ESC is causing that rather than the command signal sent here.
  for(posN = 1040; posN < 1150; posN += 1)
  {
    posE = posS = posW = posN;
    servoN.writeMicroseconds(posN);
    servoE.writeMicroseconds(posE);
    servoS.writeMicroseconds(posS);
    servoW.writeMicroseconds(posW);
    delay(300);
    digitalWrite(13,HIGH);
    Serial.println(posN);    
  } 
  return;
  for(posN = 55; posN>=50; posN-=1)
  {    
    posE = posS = posW = posN;    
    servoN.writeMicroseconds(posN);
    servoE.writeMicroseconds(posE);
    servoS.writeMicroseconds(posS);
    servoW.writeMicroseconds(posW);
    delay(500);                       // waits 15ms for the servo to reach the position 
    digitalWrite(13,LOW);
    Serial.println(posN);
  }   
} 

void doRegularMotorStart()
{
  Serial.print("Starting motors in operation mode...");
  servoN.write(30);
  servoE.write(30);
  servoS.write(30);
  servoW.write(30);
  delay(5000);
  Serial.println(" done");
}

void doThrottleRangeSet()
{
  Serial.println("--- Throttle range setting mode ---");
  Serial.println("--- Make sure the battery is disconnected when starting this mode. ---\n");
  delay(5000);
  Serial.print("Setting throttle to high ...");
  servoN.writeMicroseconds(2000);
  servoE.writeMicroseconds(2000);
  servoS.writeMicroseconds(2000);
  servoW.writeMicroseconds(2000);
  Serial.println("done");
  Serial.print("Prepare to connect battery ... ");
  Serial.print("5 ... ");
  delay(1000); Serial.print("4 ... ");
  delay(1000); Serial.print("3 ... ");
  delay(1000); Serial.print("2 ... ");
  delay(1000); Serial.print("1 ... ");
  delay(1000); Serial.println("0");
  Serial.println("Connect battery. \nAfter about 2 seconds you should hear BEEP-BEEP indicating that the high point has been set.");
  Serial.print("5 ... ");
  delay(1000); Serial.print("4 ... ");
  delay(1000); Serial.print("3 ... ");
  delay(1000); Serial.print("2 ... ");
  delay(1000); Serial.print("1 ... ");
  delay(1000); Serial.println("0");
  Serial.print("Setting throttle to low ...");
  servoN.writeMicroseconds(1000);
  servoE.writeMicroseconds(1000);
  servoS.writeMicroseconds(1000);
  servoW.writeMicroseconds(1000);
  Serial.println("done\n");
  Serial.println("You should soon hear a BEEP for every battery cell present and then a long beep confirming that the low point has been set.");

  Serial.println("/// Throttle range setting done \\\\\\");
}
