#ifndef JOYSTICK_H
#define JOYSTICK_H

#define START_BYTE 0xFE
#define STOP_BYTE 0xFF
#define Y_MASK 0x01
#define X_MASK 0x02
#define B_MASK 0x04
#define A_MASK 0x08
#define START_MASK 0x10
#define BACK_MASK 0x20

#define Y
class Joy
{
public:
    Joy();
    void setup();
    void getAxis(int *axisX, int *axisY, int*axisZ);
    bool read();
    bool getButtonA();
    bool getButtonB();
    bool getButtonX();
    bool getButtonY();
    bool getButtonStart();
    bool getButtonBack();
private:
    bool getButton(byte buttonMask);
    int m_axisX;
    int m_axisY;
    int m_axisZ;
    byte m_buttons;
    bool m_buttonA;
    bool m_buttonB;
    bool m_buttonX;
    bool m_buttonY;
    bool m_buttonStart;
    bool m_buttonBack;
};
#endif
