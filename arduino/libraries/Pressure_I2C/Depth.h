#ifndef DEPTH_H
#define DEPTH_H

#include <Arduino.h>
#include <Wire.h>

enum temperature_units
{
	CELSIUS,
	FAHRENHEIT
};

enum measurement
{
	PRESSURE    = 0x00,
	TEMPERATURE = 0x10
};

//Define constants for conversion precision

enum precision
{
	ADC_256  = 0x00,
	ADC_512  = 0x02,
	ADC_1024 = 0x04,
	ADC_2048 = 0x06,
	ADC_4096 = 0x08
}; 

enum precision_delay
{
	ADC_256_DELAY  = 2, 
	ADC_512_DELAY  = 4,
	ADC_1024_DELAY = 5,
	ADC_2048_DELAY = 7,
	ADC_4096_DELAY = 11
};

enum depth_addr
{
	ADDRESS_HIGH = 0x76,
	ADDRESS_LOW = 0x77
};

//comands
#define CMD_RESET 0x1E //reset command
#define CMD_ADC_READ 0x00 // ADC read command
#define CMD_ADC_CONV 0x40 // ADC conversion command

#define CMD_PROM 0xA0 // Coefficient location

class Depth 
{

	public: 
		Depth(depth_addr address); 
		void reset(void);	 //Reset device
		uint8_t begin(void); // Collect coefficients from sensor
		// Return calculated temperature from sensor
		float getTemperature(temperature_units units, precision _precision);
		// Return calculated pressure from sensor
		float getPressure(precision _precision);
		void getPressurePreA(precision _precision);
		void getPressurePreB(precision _precision);
		void getPressurePreC();

	private:
		int32_t _pressure_raw;
		int32_t _temperature_raw;
		int32_t _temperature_actual;
		int32_t _pressure_actual;
		depth_addr _address; 		// Variable used to store I2C device address.
		uint16_t coefficient[8];// Coefficients;
		void getMeasurements(precision _precision);
		void sendCommand(uint8_t command);	// General I2C send command function
		void getADCConversionA(measurement _measurement, precision _precision);	// Retrieve ADC result
		uint32_t getADCConversionB();
		void sensorWait(uint8_t time); // General delay function see: delay()

};

#endif


