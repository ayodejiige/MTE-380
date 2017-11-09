#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#include "joy.h"

Joy::Joy()
{
    // m_axisX = 0;
    // m_axisY = 0;
    // m_axisZ = 0;
    // m_buttons = 0;
    m_data = {0};
}

void Joy::begin(uint32_t baudrate)
{
    Serial1.begin(baudrate);
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

  // store joystick data
  m_data.axisX = buff[0];
  m_data.axisY = buff[1];
  m_data.axisZ = buff[2];
  m_data.buttonA = buff[3] & A_MASK;
  m_data.buttonB = buff[3] & B_MASK;
  m_data.buttonX = buff[3] & X_MASK;
  m_data.buttonY = buff[3] & Y_MASK;
  m_data.buttonStart = buff[3] & START_MASK;
  m_data.buttonBack = buff[3] & BACK_MASK;
  Serial.println("Finished!");
  // m_buttons = buff[3];

  return 1;
}

joy_data_t Joy::getData()
{
  joy_data_t res = m_data;

  // reset button values
  m_data.buttonA = 0;
  m_data.buttonB = 0;
  m_data.buttonX = 0;
  m_data.buttonY = 0;
  m_data.buttonStart = 0;
  m_data.buttonBack = 0;

  return res;
}
// void Joy::getAxis(int *axisX, int *axisY, int*axisZ)
// {
//     *axisX = m_axisX;
//     *axisY = m_axisY;
//     *axisZ = m_axisZ;
// }

// bool Joy::getButton(joy_constants buttonMask)
// {
//     // store button value
//     bool res = m_buttons & buttonMask;
//     // clear button value
//     m_buttons  = (~(byte)buttonMask) & m_buttons;
//     return res;
// }

// bool Joy::getButtonA()
// {
//   return getButton(A_MASK);
// }

// bool Joy::getButtonB()
// {
//   return getButton(B_MASK);
// }

// bool Joy::getButtonX()
// {
//   return getButton(X_MASK);
// }

// bool Joy::getButtonY()
// {
//   return getButton(Y_MASK);
// }

// bool Joy::getButtonStart()
// {
//   return getButton(START_MASK);
// }

// bool Joy::getButtonBack()
// {
//   return getButton(BACK_MASK);
// }

void printJoystick(joy_data_t data)
{
  Serial.print("x ");
  Serial.print(data.axisX);
  Serial.print(" y ");
  Serial.print(data.axisY);
  Serial.print(" z ");
  Serial.print(data.axisZ);
  Serial.print(" X ");
  Serial.print(data.buttonX);
  Serial.print(" Y ");
  Serial.print(data.buttonY);
  Serial.print(" A ");
  Serial.print(data.buttonA);
  Serial.print(" B ");
  Serial.print(data.buttonB);
  Serial.print(" back ");
  Serial.print(data.buttonBack);
  Serial.print(" start ");
  Serial.print(data.buttonStart);
  Serial.println("");
}
