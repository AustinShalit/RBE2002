
#pragma once

#include <PID_v1.h>

class XV11 {
public:
    XV11(uint8_t motorPin);

    void Update(uint16_t data);
    void Enable(bool enable);

private:
    uint8_t m_motorPin;
    bool m_enabled = false;

    PID m_pid;    
    double m_speed = 200.0;
    double m_motorOutput = 200.0;
    double m_speedSetpoint = 275.0;
};