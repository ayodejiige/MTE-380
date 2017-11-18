#include "Depth.h"

Depth::Depth(depth_addr address)
{
	Wire.begin();
	_address = address;
}

void Depth::reset(void)
{
	sendCommand(CMD_RESET);
}

uint8_t Depth::begin(void)
//initialize for subsequent pressure measurements
{
	uint8_t i; 
	for(i = 0; i <= 7; i++) {
		sendCommand(CMD_PROM + (i*2)); 
		Wire.requestFrom(_address,2); 
		uint8_t highByte = Wire.read(); 
		uint8_t lowByte = Wire.read();
		coefficient[i] = (highByte << 8)|lowByte;

		// Uncomment below for debugging output.

		//	Serial.print("C");

		//	Serial.print(i);

		//	Serial.print("= ");

		//	Serial.println(coefficient[i]);

	}
	return 0; 
}


float Depth::getTemperature(temperature_units units, precision _precision)

// Return a temperature reading in either F or C.

{
	getMeasurements(_precision);
	float temperature_reported;
	// If Fahrenheit is selected return the temperature converted to F
	if(units == FAHRENHEIT){
		temperature_reported = _temperature_actual / 100;
		temperature_reported = (((temperature_reported) * 9) / 5) + 32;
		return temperature_reported;
		}

	// If Celsius is selected return the temperature converted to C	
	else {
		temperature_reported = _temperature_actual / 100;
		return temperature_reported;
	}
}

float Depth::getPressure(precision _precision)
// Return a pressure reading units Pa.
{
	float pressure_reported;
	getMeasurements(_precision);
	pressure_reported = _pressure_actual;
	pressure_reported = pressure_reported / 10;
	return pressure_reported;

}

void Depth::getPressurePreA(precision _precision)
{
	getADCConversionA(TEMPERATURE, _precision);
}

void Depth::getPressurePreB(precision _precision)
{
	_temperature_raw = getADCConversionB();
	getADCConversionA(PRESSURE, _precision);
}

void Depth::getPressurePreC()
{
	_pressure_raw = getADCConversionB();
}

void Depth::getMeasurements(precision _precision)
{
	//Create Variables for calculations
	int32_t temp_calc;
	int32_t pressure_calc;

	int32_t dT;

	//Now that we have a raw temperature, let's compute our actual.
	dT = _temperature_raw - ((int32_t)coefficient[5] << 8);
	temp_calc = (((int64_t)dT * coefficient[6]) >> 23) + 2000;

	

	// TODO TESTING  _temperature_actual = temp_calc;

	//Now we have our first order Temperature, let's calculate the second order.
	int64_t T2, OFF2, SENS2, OFF, SENS; //working variables

	if (temp_calc < 2000) 

	// If temp_calc is below 20.0C
	{	

		T2 = 3 * (((int64_t)dT * dT) >> 33);
		OFF2 = 3 * ((temp_calc - 2000) * (temp_calc - 2000)) / 2;
		SENS2 = 5 * ((temp_calc - 2000) * (temp_calc - 2000)) / 8;

		
		if(temp_calc < -1500)
		// If temp_calc is below -15.0C 
		{

			OFF2 = OFF2 + 7 * ((temp_calc + 1500) * (temp_calc + 1500));
			SENS2 = SENS2 + 4 * ((temp_calc + 1500) * (temp_calc + 1500));

		}

    } 

	else
	// If temp_calc is above 20.0C
	{ 
		T2 = 7 * ((uint64_t)dT * dT)/pow(2,37);
		OFF2 = ((temp_calc - 2000) * (temp_calc - 2000)) / 16;
		SENS2 = 0;
	}

	// Now bring it all together to apply offsets 

	OFF = ((int64_t)coefficient[2] << 16) + (((coefficient[4] * (int64_t)dT)) >> 7);
	SENS = ((int64_t)coefficient[1] << 15) + (((coefficient[3] * (int64_t)dT)) >> 8);

	temp_calc = temp_calc - T2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;

	// Now lets calculate the pressure

	pressure_calc = (((SENS * _pressure_raw) / 2097152 ) - OFF) / 32768;

	_temperature_actual = temp_calc ;

	_pressure_actual = pressure_calc ; // 10;// pressure_calc;

}

void Depth::getADCConversionA(measurement _measurement, precision _precision)
{	
	sendCommand(CMD_ADC_CONV + _measurement + _precision);
	// external delay before calling get ADC conversion
}

uint32_t Depth::getADCConversionB()
{	
	uint32_t result;
	uint8_t highByte, midByte, lowByte;

	sendCommand(CMD_ADC_READ);

	Wire.requestFrom(_address, 3);

	while(Wire.available())    
	{ 
		highByte = Wire.read();
		midByte = Wire.read();
		lowByte = Wire.read();	

	}

	result = ((uint32_t)highByte << 16) + ((uint32_t)midByte << 8) + lowByte;
	return result;
}

void Depth::sendCommand(uint8_t command)

{	
	Wire.beginTransmission( _address);
	Wire.write(command);
	Wire.endTransmission();
}

