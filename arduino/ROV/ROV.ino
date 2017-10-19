/*

*/
#include <Servo.h>
#include <stdint.h>
#include <SoftwareSerial.h>

#define INPUT_SIZE 3
#define STOP 1460
#define MAX 1860
#define MIN 1060
#define MOTOR_STOP 1460
#define MOTOR_MAX 1820 //abs max is 1860
#define MOTOR_MIN 1100 //abs min is 1060

#define Y_MASK 0x01
#define X_MASK 0x02
#define B_MASK 0x04
#define A_MASK 0x08
#define START_MASK 0x10
#define BACK_MASK 0x20

Servo surgeR,surgeL,pitchT,pitchB;
int8_t x, y, z;
bool buttonA, buttonB, buttonX, buttonY, buttonStart, buttonBack, buttonL1, buttonR1, buttonL3, buttonR3 = 0;
bool killState = 0;
uint16_t Rval, Lval, Tval, Bval  = 10;
uint16_t Templarge, Tempsmall  = MOTOR_STOP;

void setup() {

  Serial.begin(9600);    // start serial at 9600 baud
  Serial1.begin(9600);   // software serial port
  surgeR.attach(6);
  surgeL.attach(9);
  pitchT.attach(10);
  pitchB.attach(11);

  surgeR.writeMicroseconds(STOP);
  delay(1000);
  surgeL.writeMicroseconds(STOP);
  delay(1000);
  pitchT.writeMicroseconds(STOP);
  delay(1000);
  pitchB.writeMicroseconds(STOP);
  delay(1000);
}
void loop() {

  if(!readJoystick()) return;
  killState ^= buttonStart;
  if(killState)
  {
    moveROV(10, 10, 10);
    return;
  }
  moveROV(x, y, z);
}

bool readJoystick()
{
  /*Read joysitck input*/
  byte buff[5];
  byte data;
  // Check start byte
  while (!Serial1.available());
  Serial1.readBytes(&data, 1);
  if(data != 0xFE) return 0;

  // read input
  while (! Serial1.available() == 5);
  Serial1.readBytes(buff, 5);

  // check stop byte
  while (!Serial1.available());
  Serial1.readBytes(&data, 1);
  if(data != 0xFF) return 0;

  x = buff[0];
  y = buff[1];
  z = buff[2];
  buttonA = buff[3] & A_MASK;
  buttonB = buff[3] & B_MASK;
  buttonX = buff[3] & X_MASK;
  buttonY = buff[3] & Y_MASK;
  buttonStart = buff[3] & START_MASK;
  buttonBack = buff[3] & BACK_MASK;
 
  return 1;
}

void printJoystick()
{
  Serial.print("x ");
  Serial.print(x);
  Serial.print(" y ");
  Serial.print(y);
  Serial.print(" z ");
  Serial.print(z);
  Serial.print(" X ");
  Serial.print(buttonX);
  Serial.print(" Y ");
  Serial.print(buttonY);
  Serial.print(" A ");
  Serial.print(buttonA);
  Serial.print(" B ");
  Serial.print(buttonB);
  Serial.print(" back ");
  Serial.print(buttonBack);
  Serial.print(" start ");
  Serial.print(buttonStart);
  Serial.println("");
}
void moveROV(int16_t x, int16_t y, int16_t z)
{
  // map input values to motor values
  if (x<=10) x = 0;
  else x = x - 10;
  
  Rval = map(x,0,10,MOTOR_STOP,MOTOR_MIN);
  Lval = Rval ,  Tval = Rval;
  Bval = map(x,0,10, MOTOR_STOP, MOTOR_MAX);
  
  
  if (y!=10)
  {
    int diffY = abs(y-10); // 0~10
    diffY = map(diffY, 0, 10, 0, 5);
    if ((x+diffY)>10)
    {
      Templarge = MOTOR_MAX;
      Tempsmall= map((x-diffY-((x+diffY)-10)),0, 10, MOTOR_STOP, MOTOR_MAX);
    }
    else if ((x-diffY)<0)
    {
      Tempsmall = MOTOR_STOP;
      Templarge = map((x+diffY-(x-diffY)), 0, 10, MOTOR_STOP, MOTOR_MAX);
    }
    else
    {
      Templarge = map(x+diffY, 0, 10, MOTOR_STOP, MOTOR_MAX);
      Tempsmall = map(x-diffY, 0, 10, MOTOR_STOP, MOTOR_MAX);
    }
    
    if (y>10)
    {
      Rval = map(Tempsmall, MOTOR_STOP, MOTOR_MAX, MOTOR_STOP, MOTOR_MIN);
      Lval = map(Templarge, MOTOR_STOP, MOTOR_MAX, MOTOR_STOP, MOTOR_MIN);
    }
    else
    {
      Rval = map(Templarge, MOTOR_STOP, MOTOR_MAX, MOTOR_STOP, MOTOR_MIN);
      Lval = map(Tempsmall, MOTOR_STOP, MOTOR_MAX, MOTOR_STOP, MOTOR_MIN);
    }
  }
  

  if (z!=10)
  {
    int diffZ = abs(z-10); // 0~10
    diffZ = map(diffZ, 0, 10, 0, 5);
    if ((x+diffZ)>10)
    {
      Templarge = MOTOR_MAX;
      Tempsmall= map((x-diffZ-((x+diffZ)-10)), 0, 10, MOTOR_STOP, MOTOR_MAX);
    }
    else if ((x-diffZ)<0)
    {
      Tempsmall = MOTOR_STOP;
      Templarge = map((x+diffZ-(x-diffZ)), 0, 10, MOTOR_STOP, MOTOR_MAX);
    }
    else
    {
      Templarge = map(x+diffZ, 0, 10, MOTOR_STOP, MOTOR_MAX);
      Tempsmall = map(x-diffZ, 0, 10, MOTOR_STOP, MOTOR_MAX);
    }
    if (z>10)
    {
      Bval = Templarge;
      Tval = map(Tempsmall, MOTOR_STOP, MOTOR_MAX, MOTOR_STOP, MOTOR_MIN);
    }
    else
    {
      Bval = Tempsmall;
      Tval = map(Templarge, MOTOR_STOP, MOTOR_MAX, MOTOR_STOP, MOTOR_MIN);
    }
  }


  if (x<=0 && y==10 && z==10)
  {//stop ROV
    Rval, Lval, Tval, Bval = MOTOR_STOP;
  }
  surgeR.writeMicroseconds(Rval);
  surgeL.writeMicroseconds(Lval);
  pitchT.writeMicroseconds(Tval);
  pitchB.writeMicroseconds(Bval);
  Serial.print("Rval "); Serial.print(Rval);
  Serial.print(" Lval "); Serial.print(Lval);
  Serial.print(" Tval "); Serial.print(Tval);
  Serial.print(" Bval "); Serial.print(Bval);
  Serial.println("");
}


