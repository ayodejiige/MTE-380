#ifndef SONAR_H
#define SONAR_H

#define READ_CMD  0x51

#if (ARDUINO >= 100)
    #include "Arduino.h"
#else
    #include "WProgram.h"
#endif


#include <Wire.h>

class Sonar
{
public:
    Sonar(uint8_t address, uint32_t window);
    bool begin();
    void getDistancePre(); 
    uint32_t getDistancePost();
private:
    uint32_t m_buffer[5];
    uint8_t m_address;
    uint32_t m_window;
    uint32_t m_idx;
    void getRangeA();
    uint32_t getRangeB();    
};
#endif