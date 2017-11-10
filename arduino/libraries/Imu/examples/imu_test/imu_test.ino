#include <Imu.h>

Imu imuDev;
imu_data_t imuData;
imu_cal_t imuCal;

void setup()
{
  Serial.begin(9600);
  imuDev.begin();
}

void loop()
{
  imuData = imuDev.getOrientation();
  /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(imuData.yaw, 4);
  Serial.print("\tY: ");
  Serial.print(imuData.pitch, 4);
  Serial.print("\tZ: ");
  Serial.print(imuData.roll, 4);

  imuDev.displayCalStatus();
  Serial.println("");
}