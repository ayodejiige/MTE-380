#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include "joy.h"

Joy::Joy()
{
    // do nothing
}

void Joy::setup()
{
    Serial1.begin(9600);
}


bool Joy::read()
{
  /*Read joysitck input*/
  byte buff[5];
  byte data;

  // Check start byte
  if(!Serial1.available()) return 0;
  Serial1.readBytes(&data, 1);
  if(data != START_BYTE) return 0;

  // read input
  while (!(Serial1.available() == 5));
  Serial1.readBytes(buff, 5);

  // check stop byte
  while (!Serial1.available());
  Serial1.readBytes(&data, 1);
  if(data != 0xFF) return 0;

  m_axisX = buff[0];
  m_axisY = buff[1];
  m_axisZ = buff[2];
  m_buttons = buff[3];

  return 1;
}

void Joy::getAxis(int *axisX, int *axisY, int*axisZ)
{
    *axisX = m_axisX;
    *axisY = m_axisY;
    *axisZ = m_axisZ;
}

bool Joy::getButton(byte buttonMask)
{
    bool res = m_buttons & buttonMask;
    m_buttons  = ~((byte)(buttonMask)) & m_buttons;
    return res;
}

bool Joy::getButtonA()
{
  return getButton(A_MASK);
}

bool Joy::getButtonB()
{
  return getButton(B_MASK);
}

bool Joy::getButtonX()
{
  return getButton(X_MASK);
}

bool Joy::getButtonY()
{
  return getButton(Y_MASK);
}

bool Joy::getButtonStart()
{
  return getButton(START_MASK);
}

bool Joy::getButtonBack()
{
  return getButton(BACK_MASK);
}


