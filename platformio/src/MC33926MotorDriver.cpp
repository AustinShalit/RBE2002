#include "MC33926MotorDriver.h"

// Constructors ////////////////////////////////////////////////////////////////

MC33926MotorDriver::MC33926MotorDriver(int M1PWMA, int M1PWMB, int M1EN, int M2PWMA, int M2PWMB, int M2EN) {

    _M1EN = M1EN;
    _M2EN = M2EN;
    _M2PWMA = M2PWMA;
    _M2PWMB = M2PWMB;
    _M1PWMA = M1PWMA;
    _M1PWMB = M1PWMB;
}
MC33926MotorDriver::MC33926MotorDriver(int M1PWMA, int M1PWMB, int M1EN, int M1FB, int M2PWMA, int M2PWMB, int M2EN, int M2FB) {

    _M1EN = M1EN;
    _M2EN = M2EN;
    _M2PWMA = M2PWMA;
    _M2PWMB = M2PWMB;
    _M1PWMA = M1PWMA;
    _M1PWMB = M1PWMB;
    _M1FB = M1FB;
    _M2FB = M2FB;
}

MC33926MotorDriver::MC33926MotorDriver(int M1PWMA, int M1PWMB, int M1EN) {
    _M1EN = M1EN;
    _M1PWMA = M1PWMA;
    _M1PWMB = M1PWMB;
}

MC33926MotorDriver::MC33926MotorDriver(int M1PWMA, int M1PWMB, int M1EN, int M1FB) {
    _M1EN = M1EN;
    _M1PWMA = M1PWMA;
    _M1PWMB = M1PWMB;
    _M1FB = M1FB;
}

// Public Methods //////////////////////////////////////////////////////////////
void MC33926MotorDriver::Init()
{
// Define pinMode for the pins and set the frequency for timer1.

    pinMode(_M1PWMA,OUTPUT);
    pinMode(_M1PWMB,OUTPUT);
    pinMode(_M1EN, OUTPUT);
    digitalWrite(_M1EN, LOW);

    pinMode(_M2PWMA,OUTPUT);
    pinMode(_M2PWMB,OUTPUT);
    pinMode(_M2EN, OUTPUT);
    digitalWrite(_M2EN,LOW);

}

// Set speed for motor 2, speed is a number between -255 and 255
void MC33926MotorDriver::SetM1Speed(int speed) {

    //If motor is set to 0, disable the output
    if(speed == 0) {
        digitalWrite(_M1EN,LOW);
        return;
    }
    digitalWrite(_M1EN, HIGH);

    //set reverse
    boolean reverse = speed < 0;
    if(reverse) speed = -speed;

    //set outputs
    if (reverse)
    {
        analogWrite(_M1PWMA, speed);
        digitalWrite(_M1PWMB, LOW);
    }
    else {
        analogWrite(_M1PWMB, speed);
        digitalWrite(_M1PWMA, LOW);
    }
}

void MC33926MotorDriver::M1Break() {
    digitalWrite(_M1EN, HIGH);
    digitalWrite(_M1PWMA, LOW);
    digitalWrite(_M1PWMB, LOW);
}

// Set speed for motor 2, speed is a number between -255 and 255
void MC33926MotorDriver::SetM2Speed(int speed) {

    //If motor is set to 0, disable the output
    if(speed == 0) {
        digitalWrite(_M2EN,LOW);
        return;
    }
    digitalWrite(_M2EN, HIGH);

    //set reverse
    boolean reverse = speed < 0;
    if(reverse) speed = -speed;

    //set outputs
    if (reverse)
    {
        analogWrite(_M2PWMA, speed);
        digitalWrite(_M2PWMB, LOW);
    }
    else {
        analogWrite(_M2PWMB, speed);
        digitalWrite(_M2PWMA, LOW);
    }
}

void MC33926MotorDriver::M2Break() {
    digitalWrite(_M2EN, HIGH);
    digitalWrite(_M2PWMA, LOW);
    digitalWrite(_M2PWMB, LOW);
}

void MC33926MotorDriver::BreakAll() {
    M1Break();
    M2Break();
}

// Set speed for motor 1 and 2
void MC33926MotorDriver::SetSpeeds(int m1Speed, int m2Speed) {
    SetM1Speed(m1Speed);
    SetM2Speed(m2Speed);
}

unsigned int MC33926MotorDriver::GetM1Current() {
    // 5V / 1024 ADC counts / 525 mV per A = 9 mA per count
    return analogRead(_M1FB) * 9;
}

unsigned int MC33926MotorDriver::GetM2Current() {
    // 5V / 1024 ADC counts / 525 mV per A = 9 mA per count
    return analogRead(_M2FB) * 9;
}

