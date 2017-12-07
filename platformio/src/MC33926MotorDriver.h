#ifndef MC33926MotorDriver_h
#define MC33926MotorDriver_h

#include <Arduino.h>

class MC33926MotorDriver
{
  public:  
    // CONSTRUCTORS
        //No feedback
    MC33926MotorDriver(int M1PWMA, int M1PWMB, int M1EN,
                       int M2PWMA, int M2PWMB, int M2EN);
        //Feedback
    MC33926MotorDriver(int M1PWMA, int M1PWMB, int M1EN, int M1FB,
                       int M2PWMA, int M2PWMB, int M2EN, int M2FB);
        //Single driver, no feedback
    MC33926MotorDriver(int M1PWMA, int M1PWMB, int M1EN);
        //Single driver, feedback
    MC33926MotorDriver(int M1PWMA, int M1PWMB, int M1EN, int M1FB);
    
    // PUBLIC METHODS
    void Init(); //Setup pins
    void SetM1Speed(int speed); // Set speed for M1.
    void M1Break();
    void SetM2Speed(int speed); // Set speed for M2.
    void M2Break();
    void SetSpeeds(int m1Speed, int m2Speed); // Set speed for both M1 and M2.
    void BreakAll();
    unsigned int GetM1Current(); // Get current reading for M1.
    unsigned int GetM2Current(); // Get current reading for M2.
    //unsigned char getFault(); // Get fault reading.
    
  private:

    int _M1PWMA;
    int _M1PWMB;
    int _M1EN;
    int _M1FB;

    int _M2PWMA;
    int _M2PWMB;
    int _M2EN;
    int _M2FB;
};

#endif