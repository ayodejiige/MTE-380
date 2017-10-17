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

bool autonomous = false;

Servo surgeR,surgeL,pitchT,pitchB;
float x, y, z;
uint16_t Rval, Lval, Tval, Bval  = 10;
uint16_t Templarge, Tempsmall  = MOTOR_STOP;
byte buff[3];

float imuPitch; // Pitch value reading from the IMU
float desiredPitch = -20;
float zSensitivy = 0.1; // smaller this is the motor output will be smaller

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

  autonomous = true; // laster this should come from the controller's button press

  if(!readJoystick()) return;
  x = buff[0];
  y = buff[1];
  z = buff[2];

  if(autonomous)
  {
    x = 0;
    y = 0;
    if(desiredPitch > imuPitch)
    {
        z = 10+zSensitivy;
    } else if (desiredPitch < imuPitch) {
        z = 10-zSensitivy;
    } else {
        z = 10;
    }
  }

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
/*
  for (int i = 0; i < 3; i++)
  {
    Serial.print(buff[i]);
    Serial.print(" ");
  }
  Serial.println(" ");
*/
  return 1;
}

void moveROV(float x, float y, float z)
{
  // map input values to motor values
  if (x<=10) x = 0;
  else x = x - 10;

  Rval = map(x,0,10,MOTOR_STOP,MOTOR_MAX);
  Lval = Rval , Tval = Rval, Bval = Rval;


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
      Rval = Templarge;
      Lval = Tempsmall;
    }
    else
    {
      Rval = Tempsmall;
      Lval = Templarge;
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
      Tval = Tempsmall;
    }
    else
    {
      Bval = Tempsmall;
      Tval = Templarge;
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

void maintainPitch()
