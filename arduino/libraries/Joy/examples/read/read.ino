#include <Joy.h>

Joy joystick;

void setup()
{
  joystick.setup();
}

void loop()
{
   joystick.read();
}
