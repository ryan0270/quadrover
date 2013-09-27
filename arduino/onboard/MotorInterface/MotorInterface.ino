#include <SPI.h>
#include <Adb.h>
#include <Wire.h>

int verbosity=0;

short motorCommands[4];

boolean phoneIsConnected;
unsigned long lastPhoneUpdateTimeMS;

class Ultrasonic
{
	public:
		Ultrasonic(int pin);
        void DistanceMeasure(void);
		long microsecondsToCentimeters(void);
		long microsecondsToInches(void);
	private:
		int _pin;//pin number of Arduino that is connected with SIG pin of Ultrasonic Ranger.
        long duration;// the Pulse time received;
};
Ultrasonic::Ultrasonic(int pin)
{
	_pin = pin;
}
/*Begin the detection and get the pulse back signal*/
void Ultrasonic::DistanceMeasure(void)
{
    pinMode(_pin, OUTPUT);
	digitalWrite(_pin, LOW);
	delayMicroseconds(2);
	digitalWrite(_pin, HIGH);
	delayMicroseconds(5);
	digitalWrite(_pin,LOW);
	pinMode(_pin,INPUT);
	duration = pulseIn(_pin,HIGH);
}
/*The measured distance from the range 0 to 400 Centimeters*/
long Ultrasonic::microsecondsToCentimeters(void)
{
	return duration/29/2;	
}

// Adb connection.
Connection * connection;
// Event handler for the shell connection. 
void adbEventHandler(Connection * connection, adb_eventType event, uint16_t length, uint8_t * data)
{
  if(event == ADB_CONNECT && verbosity > 0)
  {
    Serial.println("ADB_CONNECT");
  }
  else if(event == ADB_DISCONNECT)
  {
    phoneIsConnected = false;
    if(verbosity > 0)
      Serial.println("ADB_DISCONNECT");
  }
  else if(event == ADB_CONNECTION_OPEN && verbosity > 0)
  {
    Serial.println("ADB_CONNECTION_OPEN");
  }
  else if(event == ADB_CONNECTION_CLOSE)
  {
    phoneIsConnected = false;
    if(verbosity > 0)
      Serial.println("ADB_CONNECTION_CLOSE");
  }
  else if(event == ADB_CONNECTION_FAILED)
  {
    phoneIsConnected = false;
    if(verbosity > 0)
      Serial.println("ADB_CONNECTION_FAILED");
  }
  else if (event == ADB_CONNECTION_RECEIVE)
  {
    phoneIsConnected = true;
    for(int i=0; i<4; i++)
    {
//      byte val = data[i];
      short val = (data[2*i+1] << 8) | (data[2*i]);
      motorCommands[i] = val;
    }
    
    lastPhoneUpdateTimeMS = millis();
  }
}

boolean sendCommand(byte addr, short cmd)
{
//  Wire.beginTransmission(addr);
//  byte upper = floor(cmd/8.0);
//  byte lower = cmd % 8;
//  Wire.write(upper);
//  Wire.write( lower & 0x07 );
//  byte result = Wire.endTransmission(true);
//  return result == 0;
}
 
byte MOTOR_ADDR_E = (0x53 + (0 << 1)) >> 1;
byte MOTOR_ADDR_W = (0x53 + (1 << 1)) >> 1;
byte MOTOR_ADDR_S = (0x53 + (2 << 1)) >> 1;
byte MOTOR_ADDR_N = (0x53 + (3 << 1)) >> 1;

#define SONAR_PIN 4
Ultrasonic ultrasonic(SONAR_PIN);
byte motorAddr[4];
void setup()
{ 
  if(verbosity > 0)
  {
    Serial.begin(57600);
    Serial.println("Start chadding");
  }
  
  delay(500);
  
  // Initialise the ADB subsystem.  
  ADB::init();
  // Open an ADB stream to the phone's shell. Auto-reconnect
  connection = ADB::addConnection("tcp:45670", true, adbEventHandler);  
  
  delay(1000);
  
  Wire.begin();
  motorAddr[0] = MOTOR_ADDR_N;
  motorAddr[1] = MOTOR_ADDR_E;
  motorAddr[2] = MOTOR_ADDR_S;
  motorAddr[3] = MOTOR_ADDR_W;
    
  for(int i=0; i<4; i++)
  {
    motorCommands[i] = 0;
    sendCommand(motorAddr[i], 0);
  }
    
  doRegularMotorStart();

  phoneIsConnected = false;
  lastPhoneUpdateTimeMS = millis();
}  

enum
{
  COMM_ARDUINO_HEIGHT=1,
};

unsigned long lastHeightSendTimeMS = 0;
void loop() 
{
  if((millis() - lastPhoneUpdateTimeMS) > 100)
  {
    if(phoneIsConnected && verbosity >= 1)
      Serial.println("Lost the phone");
    phoneIsConnected = false;
  }
    
  for(int i=0; i<4; i++)
  {
    if(!phoneIsConnected)
      motorCommands[i] = 0;
      
    sendCommand(motorAddr[i], motorCommands[i]);
    if(verbosity >=2 )
    {
      Serial.print(motorCommands[i]);
      Serial.print("\t");
    }
  }
  if(verbosity >= 2)
  {
    if(phoneIsConnected)
      Serial.print("***");
    Serial.print("\n");
  }
    
  if(millis()-lastHeightSendTimeMS > 50 && phoneIsConnected)
  {
    lastHeightSendTimeMS = millis();
    
    long range;
    ultrasonic.DistanceMeasure();// get the current signal time;
    range = ultrasonic.microsecondsToCentimeters();//convert the time to centimeters
    
    uint8_t code = COMM_ARDUINO_HEIGHT;
    uint16_t height = range;
    // ADB comm seems to wait for an ok reply which is needed
    // before it will send again. I don't want to wait for that
    // So I'll build everything into a single send
    uint8_t buff[3];
    buff[0] = code;
    memcpy(&(buff[1]),&height,2);
    connection->write(3,&(buff[0]));
  }
  
  while(ADB::poll());
  delay(1);
} 

void doRegularMotorStart()
{
  if(verbosity > 0)
    Serial.print("Starting motors in operation mode...");
  for(int i=0; i<500; i++)
  {
    // need to keep the comm alive
    for(int mdl=0; mdl<4; mdl++)
      sendCommand(motorAddr[mdl], 0);
    delay(10);
  }
  if(verbosity > 0)
    Serial.println(" done");
}

