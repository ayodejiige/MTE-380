#include <Joy.h>

Joy joystick;
joy_data_t joystickData;

void setup()
{
  joystick.begin(9600);
  Serial.begin(9600);
}

void loop()
{
  if(joystick.read())
  {
    joystickData = joystick.getData();
    printJoystick(joystickData);
  }
}
