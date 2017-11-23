#include "Imu.h"

Imu::Imu()
{
    m_data = {0};
    m_cal = {0};
    m_sensor = {0};
    m_refCaptured = false;
    m_bno = Adafruit_BNO055(55);
}

bool Imu::begin()
{
    if(!m_bno.begin())
    {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      while(1);
    }
    displaySensorDetails();
    displaySensorStatus();
    m_bno.setExtCrystalUse(true);
}

imu_data_t Imu::getOrientation()
{
    imu_data_t ret;
    sensors_event_t event;
    m_bno.getEvent(&event);

    m_data.yaw = event.orientation.x;
    m_data.roll = event.orientation.y;
    m_data.pitch = event.orientation.z;
    ret = m_data;
    if(m_refCaptured)
    {
      ret.yaw -= m_reference.yaw;
      ret.yaw += (abs(ret.yaw) > 180) * ((ret.yaw < 0) * 360 + (ret.yaw > 0) * (-360));
    }

    return ret;
}

imu_cal_t Imu::getCalStatus()
{
    imu_cal_t cal = {0};
    m_bno.getCalibration(&(cal.sys), &(cal.gyro), &(cal.accel), &(cal.mag));
    m_cal = cal;

    return m_cal;
}   

bool Imu::getReference()
{
  imu_cal_t cal = getCalStatus();

  getOrientation();
  m_reference.yaw = m_data.yaw;
  m_reference.pitch = m_data.pitch;
  m_reference.roll = m_data.roll;
  m_refCaptured = true;

  return true;
}

void Imu::displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  imu_cal_t cal = getCalStatus();

  Serial.print("\t");
  if (!cal.sys)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(cal.sys, DEC);
  Serial.print(" G:");
  Serial.print(cal.gyro, DEC);
  Serial.print(" A:");
  Serial.print(cal.accel, DEC);
  Serial.print(" M:");
  Serial.print(cal.mag, DEC);
}

void Imu::displaySensorStatus()
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  m_bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

void Imu::displaySensorDetails()
{
  m_bno.getSensor(&m_sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(m_sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(m_sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(m_sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(m_sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(m_sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(m_sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
