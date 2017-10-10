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

Servo surgeR,surgeL,pitchT,pitchB;
uint8_t x, y, z;
uint16_t Rval, Lval, Tval, Bval  = 10;
byte buff[3];

void setup() {

  Serial.begin(9600);    // start serial at 9600 baud
  Serial1.begin(9600);   // software serial port
  surgeR.attach(6);
  surgeL.attach(9);
  pitchT.attach(10);
  pitchB.attach(11);

  surgeR.writeMicroseconds(STOP);
  surgeL.writeMicroseconds(STOP);
  pitchT.writeMicroseconds(STOP);
  pitchB.writeMicroseconds(STOP);
  delay(1000);
}
void loop() {

  if(!readJoystick()) return;
  x = buff[0];
  y = buff[1];
  z = buff[2];

  moveROV(x, y, z);
}

bool readJoystick()
{
  /*Read joysitck input*/
  byte data;
  // Check start byte
  while (!Serial1.available());
  Serial1.readBytes(&data, 1);
  if(data != 0xFE) return 0;

  // read input
  while (! Serial1.available() == 3);
  Serial1.readBytes(buff, 3);

  // check stop byte
  while (!Serial1.available());
  Serial1.readBytes(&data, 1);
  if(data != 0xFF) return 0;

  for (int i = 0; i < 3; i++)
  {
    Serial.print(buff[i]);
    Serial.print(" ");
  }
  Serial.println(" ");

  return 1;
}

void moveROV(uint16_t x, uint16_t y, uint16_t z)
{
  // map input values to motor values
  Rval = map(x,0,20,MOTOR_MIN,MOTOR_MAX);
  Lval, Tval, Bval = Rval;
  if (y>10)
    Lval= map(y,0,20,MOTOR_MIN,MOTOR_MAX);
  else if (y<10)
    Rval= map(y,0,20,MOTOR_MIN,MOTOR_MAX);

  if (z>10)
    Bval = map(z,0,20,MOTOR_MIN,MOTOR_MAX);
  else if (z<10)
    Tval = map(z,0,20,MOTOR_MIN,MOTOR_MAX);

  if (x<=10 && y<=10 && z<=10)
  {//stop ROV
    Rval, Lval, Tval, Bval = MOTOR_STOP;
  }
  surgeR.writeMicroseconds(Rval);
  surgeL.writeMicroseconds(Lval);
  pitchT.writeMicroseconds(Tval);
  pitchB.writeMicroseconds(Bval);
}
