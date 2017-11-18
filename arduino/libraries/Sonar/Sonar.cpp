#include "Sonar.h"

int compare(const void* p1, const void* p2)
{
    uint32_t a = *((uint32_t*)p1);
    uint32_t b = *((uint32_t*)p2);

    if(a < b) return -1;
    if(a == b) return 0;
    if(a > b) return 1;

}

Sonar::Sonar(uint8_t address, uint32_t window)
{
    m_address = address;
    m_window = 5;
    m_idx = 0;
    for(int i = 0; i < m_window; i++)
    {
        m_buffer[i] = 0;
    }
}

bool Sonar::begin()
{
    Wire.begin();
    Wire.setClock(50000);

    return 1;
}

void Sonar::getDistancePre()
{
    // Get sensor reading
    getRangeA();
}
uint32_t Sonar::getDistance()
{
    m_buffer[m_idx] = getRangeB();
    m_idx = (m_idx+1)%m_window;

    // Median filter
    qsort(m_buffer, m_window, sizeof(uint32_t), compare);

    return m_buffer[m_window/2];
}

void Sonar::getRangeA()
{
  Wire.beginTransmission(m_address);
  Wire.write(READ_CMD);
  Wire.endTransmission();
}

uint32_t Sonar::getRangeB()
{
  uint32_t range = 0;
  byte range_highbyte = 0;
  byte range_lowbyte = 0;

  Wire.requestFrom((int)m_address,2);
  range_highbyte = Wire.read();
  range_lowbyte = Wire.read();

  range = ((uint32_t)range_highbyte * 256) + (uint32_t)range_lowbyte;
  return range;
}