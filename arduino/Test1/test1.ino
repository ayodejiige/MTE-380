/*

*/
#include <Servo.h>
#include <stdint.h>
#include <SoftwareSerial.h>
#include <Imu.h>
#include <Joy.h>
#include <Sonar.h>
#include <TaskScheduler.h>

#define TEST_MODE

#define INPUT_SIZE 3
#define STOP 1460
#define MAX 1860
#define MIN 1060
#define MOTOR_STOP 1460
#define MOTOR_MAX 1820 //abs max is 1860
#define MOTOR_MIN 1100 //abs min is 1060
#define THROTTLE_RANGE 50
#define TURN_RANGE 25

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


// Autonomy Test Macros
#define TEST_DEPTH 0
#define TEST_FORWARD 1
#define TEST_ROTATE 2
#define TEST_RESET_YAW 3
#define SONARX 4
#define YCOURSE2 5
#define YCOURSE3 6


// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins

// IMU Objects
Imu imuDev;
imu_data_t imuData = {0};
imu_cal_t imuCal = {0};
float yawTarget = 0;

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
int8_t x = THROTTLE_RANGE;
int8_t y = THROTTLE_RANGE;
int8_t z = THROTTLE_RANGE;

int8_t ySensitivity = 5, Sensitivity = 1;
bool killState = 0;
uint16_t Rval, Lval, Tval, Bval  = THROTTLE_RANGE;
uint16_t Templarge, Tempsmall = MOTOR_STOP;
bool autonomousMode = 0;

// States
uint16_t masterState;
uint16_t autonomyState;

// Callbacks
void sonarCallback();
void updateSensors();
void updateJoystick();
void master();
void autonomyRoutine();
void teleopRoutine();
void imuReferene();
void moveYaw();


// Tasks
Task t0(10, TASK_FOREVER, &master);
Task t1(100, TASK_FOREVER, &updateSensors);
Task t2(10, TASK_FOREVER, &updateJoystick);


// Scheduler
Scheduler runner;

void setup() {
    Serial.begin(115200);    // start serial at 9600 baud

    // State default
    masterState = 0;
    autonomyState = 3;

    // Tasks setup
    runner.init();
    runner.addTask(t0);
    runner.addTask(t1);
    runner.addTask(t2);
    delay(5000);
    t0.enable();
    t1.enable();
    t2.enable();

    // Joystick setup
    joystick.begin(115200);   // software serial port

    //IMU setup
    imuDev.begin();

    // Sonar setup
    sonarX.begin();
    sonarY.begin();

    // Motor setup
    surgeR.attach(8);
    pitchT.attach(9);
    pitchB.attach(10);
    surgeL.attach(11);

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

void moveYaw(float targetYaw)
{
    x = THROTTLE_RANGE;
    z = THROTTLE_RANGE;
    float deltaYaw = imuData.yaw - targetYaw;

    if(deltaYaw > 5) y = THROTTLE_RANGE - ySensitivity;
    else if(deltaYaw < -5) y = THROTTLE_RANGE + ySensitivity;
    else y = THROTTLE_RANGE;
}

// Front sonar readings
void updateDeltaX()
{
  absSonarX = sonarDataX*cos((90-imuData.yaw*3.14/180));
}

// Side sonar readings
void updateDeltaY()
{
  absSonarY = sonarDataY*sin((90-imuData.yaw)*3.14/180);
}

void updateJoystick()
{
    if(joystick.read()) 
    {
      joystickData = joystick.getData();
      switch(masterState)
      {
        case STATE_INIT:
          masterState = joystickData.buttonA ? STATE_IMU_REF : masterState;
          break;
        case STATE_TELEOP:
          masterState = joystickData.buttonX ?  STATE_AUTONOMY : masterState;
          break;
        case STATE_AUTONOMY:
          masterState = joystickData.buttonX ?  STATE_TELEOP : masterState;
          if(joystickData.buttonB)
          {
            autonomyState = (autonomyState + 1) % 7;
            x = y = z = THROTTLE_RANGE; 
            Serial.print("autonomyState: ");
            Serial.println(autonomyState);
          }
          break;
        default:
          break;
      }
    }
    
}
    

void updateSensors()
{
    sonarX.getDistancePre();
    sonarY.getDistancePre();
    t1.setCallback(&sonarCallback);
    t1.delay(80);
    imuData = imuDev.getOrientation();
    imuCal = imuDev.getCalStatus();
    sendSensorData(sonarDataX, sonarDataY, imuData.yaw, imuData.pitch, imuData.roll, imuCal.sys, imuCal.gyro, imuCal.mag);
}

void sonarCallback()
{
  sonarDataX = sonarX.getDistancePost();
  sonarDataY = sonarY.getDistancePost();
  t1.setCallback(&updateSensors);
}

void sendSensorData(float sonarX, float sonarY, float yaw, float pitch, float roll, int sys, int gyr, int mag)
{
    String first = "\t";
    String del = ",";
    String last = "\n";
    String msg = first + String(masterState) + String(autonomyState)+ del + String(sonarX) + del + String(sonarY) + del + String(yaw, 3) + del + String(pitch, 3) + del 
            + String(roll, 3) + del + String(sys) + String(gyr)+ String(mag) + last;
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
      if(imuDev.getReference()) masterState = STATE_TELEOP;
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
    case TEST_DEPTH:
        // pressure sensor trying to achieve the pitch
        break;
    case TEST_FORWARD:
        x = THROTTLE_RANGE + 10;
        y = THROTTLE_RANGE;
        z = THROTTLE_RANGE;
        if (abs(imuData.yaw) > 5)
        {
            moveYaw(0);
        }
        break;
    case TEST_ROTATE:
        moveYaw(30);
        break;
    case TEST_RESET_YAW:
        moveYaw(0);
        break;
    case SONARX:
      updateDeltaX();
      if (absSonarX > SONARX_MAX)
      {
        moveYaw(0);
        x = THROTTLE_RANGE+10;        
      }
      else if (absSonarX < SONARX_MAX)
      {
        x = THROTTLE_RANGE;
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
        moveYaw(0);
        x = THROTTLE_RANGE+10;
      }
      break;

    case YCOURSE3:
      updateDeltaX();
      if (absSonarX > SONARY_MAX)
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
        moveYaw(0);
        x = THROTTLE_RANGE+10;
      }
      break;
  }
}


void teleopRoutine()
{
  x = joystickData.axisX;
  y = joystickData.axisY;
  z = joystickData.axisZ;
}

void moveROV(int16_t x, int16_t y, int16_t z)
{
  // map input values to motor values
  if (x<=50) x = 0;
  else x = x - THROTTLE_RANGE;

  Rval = map(x,0,THROTTLE_RANGE,MOTOR_STOP,MOTOR_MIN);
  Lval = Rval , Tval = Rval;
  Bval = map(x,0,THROTTLE_RANGE, MOTOR_STOP, MOTOR_MAX);

  if (y!=THROTTLE_RANGE)
  {
    int diffY = abs(y-THROTTLE_RANGE); // 0~10
    diffY = map(diffY, 0, THROTTLE_RANGE, 0, TURN_RANGE);
    if ((x+diffY)>THROTTLE_RANGE)
    {
      Templarge = MOTOR_MAX;
      Tempsmall= map((x-diffY-((x+diffY)-THROTTLE_RANGE)),0, TURN_RANGE, MOTOR_STOP, MOTOR_MAX);
    }
    else if ((x-diffY)<0)
    {
      Tempsmall = MOTOR_STOP;
      Templarge = map((x+diffY-(x-diffY)), 0, THROTTLE_RANGE, MOTOR_STOP, MOTOR_MAX);
    }
    else
    {
      Templarge = map(x+diffY, 0, THROTTLE_RANGE, MOTOR_STOP, MOTOR_MAX);
      Tempsmall = map(x-diffY, 0, THROTTLE_RANGE, MOTOR_STOP, MOTOR_MAX);
    }

    if (y>THROTTLE_RANGE)
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

  if (z!=THROTTLE_RANGE)
  {
    int diffZ = abs(z-THROTTLE_RANGE); // 0~10
    diffZ = map(diffZ, 0, THROTTLE_RANGE, 0, TURN_RANGE);
    if ((x+diffZ)>THROTTLE_RANGE)
    {
      Templarge = MOTOR_MAX;
      Tempsmall= map((x-diffZ-((x+diffZ)-THROTTLE_RANGE)), 0, THROTTLE_RANGE, MOTOR_STOP, MOTOR_MAX);
    }
    else if ((x-diffZ)<0)
    {
      Tempsmall = MOTOR_STOP;
      Templarge = map((x+diffZ-(x-diffZ)), 0, THROTTLE_RANGE, MOTOR_STOP, MOTOR_MAX);
    }
    else
    {
      Templarge = map(x+diffZ, 0, THROTTLE_RANGE, MOTOR_STOP, MOTOR_MAX);
      Tempsmall = map(x-diffZ, 0, THROTTLE_RANGE, MOTOR_STOP, MOTOR_MAX);
    }
    if (z>THROTTLE_RANGE)
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


  if (x<=0 && y==THROTTLE_RANGE && z==THROTTLE_RANGE)
  {//stop ROV
    Rval, Lval, Tval, Bval = MOTOR_STOP;
  }
  surgeR.writeMicroseconds(Rval);
  surgeL.writeMicroseconds(Lval);
  pitchT.writeMicroseconds(Tval);
  pitchB.writeMicroseconds(Bval);
 // Serial.println("Rval "); Serial.print(Rval);
//  Serial.print(" Lval "); Serial.print(Lval);
//  Serial.print(" Tval "); Serial.print(Tval);
//  Serial.print(" Bval "); Serial.print(Bval);
//  Serial.println("");
} 
