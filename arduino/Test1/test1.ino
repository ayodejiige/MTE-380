/*

*/
#include <Servo.h>
#include <stdint.h>
#include <SoftwareSerial.h>
#include <Imu.h>
#include <Joy.h>
#include <Sonar.h>
#include <TaskScheduler.h>

#define INPUT_SIZE 3
#define STOP 1460
#define MAX 1860
#define MIN 1060
#define MOTOR_STOP 1460
#define MOTOR_MAX 1820 //abs max is 1860
#define MOTOR_MIN 1100 //abs min is 1060

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins

// IMU Objects
Imu imuDev;
imu_data_t imuData;
imu_cal_t imuCal;
bool imuReset = 0;
bool imuIsReset = 0;
float yawZero = -180;

// Joystick
Joy joystick;
joy_data_t joystickData;

// Sonar
Sonar sonar = Sonar(0x70, 5);
uint32_t sonarData = 0;

// Motor
Servo surgeR,surgeL,pitchT,pitchB;
int8_t x, y, z;
int8_t ySensitivity = 1; 
bool killState = 0;
uint16_t Rval, Lval, Tval, Bval  = 10;
uint16_t Templarge, Tempsmall = MOTOR_STOP;
bool autonomousMode = 0;

// Callbacks
void updateSensors();
void updateJoystick();
void imuReferene();
void moveYaw();

// Tasks
Task t0(100, TASK_FOREVER, &imuReference);
Task t1(10, TASK_FOREVER, &updateSensors);
Task t2(5, TASK_FOREVER, &updateJoystick);

// Scheduler
Scheduler runner;

void setup() {
    Serial.begin(9600);    // start serial at 9600 baud

    // Tasks setup
    runner.init();
    runner.addTask(t1);
    runner.addTask(t2);
    delay(5000);
    t1.enable();
    t2.enable();

    // Joystick setup
    joystick.begin(9600);   // software serial port

    //IMU setup
    imuDev.begin();

    // Sonar setup
    sonar.begin();

    // Motor setup
    surgeR.attach(8);
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
    runner.execute();

    killState ^= joystickData.buttonStart;
    autonomousMode ^= joystickData.buttonX;

    if(killState)
    {
        moveROV(10, 10, 10); // kill
        return;
    }

    if(autonomousMode)
    {
        // do nothing
    }
    else
    {
        moveROV(joystickData.axisX, joystickData.axisY, joystickData.axisZ);
    }

//    Serial.print("Autonomus: ");
//    Serial.print(autonomousMode);
//    Serial.print("\tKill: ");
//    Serial.print(killState);
//    Serial.println("");
}

void moveYaw(float requiredYaw)
{
    x = 10;
    z = 10;

    float deltaYaw = requiredYaw - imuData.yaw ;

    if(deltaYaw > 0) y = 10 + ySensitivity;
    if(deltaYaw < 0) y = 9 - ySensitivity;
}

void imuReferene()
{
    if(imuReset && !imuIsReset)
    {
        // get reference
        yawZero = imuData.yaw;
    }
}

void updateJoystick()
{
    joystick.read();
    joystickData = joystick.getData();
    imuReset ^= joystickData.buttonY;
}

void updateSensors()
{
    sonarData = sonar.getDistance();
    imuData = imuDev.getOrientation();
    imuCal = imuDev.getCalStatus();
    sendSensorData(sonarData, imuData.yaw, imuData.pitch, imuData.roll);
}

void sendSensorData(float sonar, float yaw, float pitch, float roll)
{
    String first = "\t";
    String del = ",";
    String last = "\n";

    String msg = first + String(sonar) + del + String(yaw, 3) +
        del + String(pitch, 3) + del + String(roll, 3) + last;

    Serial1.print(msg);
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
//  Serial.print("Rval "); Serial.print(Rval);
//  Serial.print(" Lval "); Serial.print(Lval);
//  Serial.print(" Tval "); Serial.print(Tval);
//  Serial.print(" Bval "); Serial.print(Bval);
//  Serial.println("");
}
