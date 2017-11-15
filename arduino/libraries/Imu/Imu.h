#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)

typedef struct
{
    float yaw;
    float pitch;
    float roll;
} imu_data_t;

typedef struct
{
    uint8_t sys;
    uint8_t gyro;
    uint8_t accel;
    uint8_t mag;
} imu_cal_t;

class Imu
{
public:
    Imu();
    bool begin();
    imu_data_t getOrientation();
    imu_cal_t getCalStatus();
    bool getReference();
    void displaySensorDetails();
    void displaySensorStatus();
    void displayCalStatus();

private:
    imu_data_t m_data;
    imu_data_t m_reference;
    imu_cal_t m_cal;
    sensor_t m_sensor;
    bool m_refCaptured;
    Adafruit_BNO055 m_bno;
};
#endif
