#include <stdint.h>
#include <SoftwareSerial.h>
#include <SoftI2CMaster.h> //You will need to install this library 
#define SCL_PIN 5 //Default SDA is Pin5 PORTC for the UNO -- you can set this to any tristate pin
#define SCL_PORT PORTC
#define SDA_PIN 4 //Default SCL is Pin4 PORTC for the UNO -- you can set this to any tristate pin
#define SDA_PORT PORTC
#define I2C_TIMEOUT 100 

void setup(){
 // Initialize both the serial and I2C bus
 Serial.begin(9600);
 i2c_init();
 // (OPTIONAL) Check each address for a sensor
 //address_polling_example();
 /*
 Note that I placed the address change example in setup() for a good reason.
 Changing the sensor address causes an EEPROM write, there should only be ~1,000,000+
 of these writes to the sensor microcontroller over its product lifetime.
 Changing the address is fine, but doing it every second for the next 4 years may
 cause reliability issues.
 */
 // (OPTIONAL) Run an address change example
 //default_address_change_example();
 // Your code here
} 

void loop()
{
 // (OPTIONAL) Read a sensor at the default address
 read_the_sensor_example();
 delay(500);
 // Your code here
} 

void read_the_sensor_example(){
 boolean error = 0; //Create a bit to check for catch errors as needed.
 int range;

 //Take a range reading at the default address of 224
 error = start_sensor(224); //Start the sensor and collect any error codes.
 if (!error){ //If you had an error starting the sensor there is little point in reading it as you
 //will get old data.
 delay(100);
 range = read_sensor(224); //reading the sensor will return an integer value -- if this value is 0 there was
 //an error
 Serial.print("R:");Serial.println(range);
 }
} 

