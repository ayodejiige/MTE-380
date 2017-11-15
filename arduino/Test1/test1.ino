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

// Sonar values UNCHARACTERIZED
#define SONARX_MAX 50
#define SONARX_MIN 23
#define SONARY_MAX 140
#define SONARY_MIN 23
#define SONARY_COURSE2 26 
#define SONARY_COURSE3 70

// Master state macros
#define STATE_INIT 0
#define STATE_IMU_REF 1
#define STATE_AUTONOMY 2
#define STATE_TELEOP 3

#define SONARX 3
#define YCOURSE2 4
#define YCOURSE3 5

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
Sonar sonarX = Sonar(0x70, 5);
Sonar sonarY = Sonar(0x71, 5); // MADE UP ADDRESS - GK
uint32_t sonarDataX = 0;
uint32_t sonarDataY = 0;
uint32_t absSonarX = 0;
uint32_t absSonarY = 0;

// Motor
Servo surgeR,surgeL,pitchT,pitchB;
int8_t x, y, z;
int8_t ySensitivity = 1;
bool killState = 0;
uint16_t Rval, Lval, Tval, Bval  = 10;
uint16_t Templarge, Tempsmall = MOTOR_STOP;
bool autonomousMode = 0;

// States
uint16_t masterState;
uint16_t autonomyState;

// Callbacks
void updateSensors();
void updateJoystick();
void master();
void autonomyRoutine();
void teleopRoutine();
void imuReferene();
void moveYaw();


// Tasks
Task t0(1, TASK_FOREVER, &master);
Task t1(10, TASK_FOREVER, &updateSensors);
Task t2(5, TASK_FOREVER, &updateJoystick);


// Scheduler
Scheduler runner;

void setup() {
    Serial.begin(9600);    // start serial at 9600 baud
    
    // State default
    masterState = 0;
    autonomyState = 0;
    
    // Tasks setup
    runner.init();
    runner.addTask(t1);
    runner.addTask(t2);
    delay(5000);
    t0.enable();
    t1.enable();
    t2.enable();

    // Joystick setup
    joystick.begin(9600);   // software serial port

    //IMU setup
    imuDev.begin();

    // Sonar setup
    sonarX.begin();
    sonarY.begin();

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

// Front sonar readings
void updateDeltaX()
{
  absSonarY = sonarDataX*cos((90-imuData.yaw*3.14/180));
}

// Side sonar readings
void updateDeltaY()
{
  absSonarY = sonarDataY*sin((90-imuData.yaw)*3.14/180);
}

void updateJoystick()
{
    joystick.read();
    joystickData = joystick.getData();
}

void updateSensors()
{
    sonarDataX = sonarX.getDistance();
    sonarDataY = sonarY.getDistance();
    imuData = imuDev.getOrientation();
    imuCal = imuDev.getCalStatus();
    sendSensorData(sonarDataX, sonarDataY, imuData.yaw, imuData.pitch, imuData.roll);
}

void sendSensorData(float sonarX, float sonarY, float yaw, float pitch, float roll)
{
    String first = "\t";
    String del = ",";
    String last = "\n";


    String msg = first + String(sonarX) + del + String(sonarY) + del + String(yaw, 3) +
        del + String(pitch, 3) + del + String(roll, 3) + last;

    Serial1.print(msg);
}

void master()
{
  switch(masterState)
  {
    case STATE_INIT:
      // nothing
      break;
    case STATE_IMU_REF:
      // capture ref function
      masterState = STATE_INIT;
      break;
    case STATE_AUTONOMY:
      autonomyRoutine();
      break;
    case STATE_TELEOP:
      teleopRoutine();
      break;
  }
  moveROV(x, y, z);
}

void autonomyRoutine()
{
  switch(autonomyState)
  {
    default:
      Serial.println("Autonomy State");
    break;
    case SONARX:
      updateDeltaX();
      if (absSonarX > SONARX_MAX)
      {
        moveYaw(yawZero);
        x = 20;        
      }
      else if (absSonarX < SONARX_MAX)
      {
        x = 10;
        moveYaw(-50);
      }
    break;

    case YCOURSE2:
      updateDeltaX();
      if (absSonarX > SONARY_MAX)
      {
        moveYaw(80);        
      }
      else if (absSonarX > SONARY_COURSE2)
      {
        moveYaw(50);
      }
      else if (absSonarX < SONARY_MIN)
      {
        moveYaw(-50);
      }
      else 
      {
        moveYaw(yawZero);
        x = 15;
      }
    break;

    case YCOURSE3:
      updateDeltaX();
      if (absSonarY > SONARY_MAX)
      {
        moveYaw(80);        
      }
      else if (absSonarX > SONARY_COURSE3+3)
      {
        moveYaw(50);        
      }
      else if (absSonarX < SONARY_COURSE3-3)
      {
        moveYaw(-50);
      }
      else if (absSonarX < SONARY_MIN)
      {
        moveYaw(-80);
      }
      else 
      {
        moveYaw(yawZero);
        x = 15;
      }
    break;
  }
}


void teleopRoutine()
{
  x = joystickData.axisX;
  y = joystickData.axisY;
  z = joystickData.axisY;
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
