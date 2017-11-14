#ifndef JOYSTICK_H
#define JOYSTICK_H

typedef struct
{
    uint16_t axisX;
    uint16_t axisY;
    uint16_t axisZ;

    bool buttonA;
    bool buttonB;
    bool buttonX;
    bool buttonY;
    bool buttonStart;
    bool buttonBack;
} joy_data_t;

void printJoystick(joy_data_t data);

class Joy
{
public:
    Joy();
    void begin(uint32_t baudrate);
    bool read();
    joy_data_t getData();
    
    // bool getButtonA();
    // bool getButtonB();
    // bool getButtonX();
    // bool getButtonY();
    // bool getButtonStart();
    // bool getButtonBack();
private:
    // bool getButton(joy_constants buttonMask);

    typedef enum
    {
        START_BYTE = 0xFE,
        STOP_BYTE  = 0xFF,
        Y_MASK     = 0x01,
        X_MASK     = 0x02,
        B_MASK     = 0x04,
        A_MASK     = 0x08,
        START_MASK = 0x10,
        BACK_MASK  = 0x20,
    }joy_constants_t;

    joy_data_t m_data;
    // int m_axisX;
    // int m_axisY;
    // int m_axisZ;
    // byte m_buttons;
};
#endif
