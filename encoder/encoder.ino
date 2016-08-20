#include <Wire.h>

//#define DEBUG

volatile byte ticks;

void setup() {
  // put your setup code here, to run once:
  //pinMode(12, INPUT);      // sets pin A as input
  //digitalWrite(12, LOW);  // turn on pullup resistors
  //pinMode(13, INPUT);      // sets pin B as input
 // digitalWrite(13, LOW);  // turn on pullup resistors
  DDRB &= B11001111;
  
  attachInterrupt(digitalPinToInterrupt(3), interrupt, RISING);

  Wire.begin(8);
  Wire.onRequest(i2cSendRotations);

#ifdef DEBUG
  Serial.begin(115200);
#endif
}

void i2cReceive(int bytes)
{
  if(bytes == 1)
  {
    byte msg = Wire.read();
    ticks = 0;
  }
}

void i2cSendRotations()
{
  int old_ticks = ticks;
  
  Wire.write(old_ticks);

  // don't reset the tick counter if we haven't accumulated
  // any rotations yet
  if(old_ticks > 0){
    ticks = 0;
  }
}

void interrupt()
{
  byte b_set = PINB & B00010000;   // read the input pin
 
  // and adjust counter + if A leads B
  ticks++; //-= b_set ? -1 : +1;
#ifdef DEBUG
  Serial.println(ticks, DEC);
#endif 
}

void loop() {
  // put your main code here, to run repeatedly:
  //delay(100);

}
