
#include <StandardCplusplus.h>
#include <algorithm>
#include <Arduino.h>

#include "XV11.h"

/**
 * Create a new XV11.
 */
XV11::XV11(uint8_t motorPin)
    : m_motorPin(motorPin), m_pid(&m_speed, &m_motorOutput, &m_speedSetpoint, 2.0, 0.5, 0.0, DIRECT) {
    m_pid.SetOutputLimits(190, 220);
    m_pid.SetMode(AUTOMATIC);
}

void XV11::Update(uint16_t data) {
    if (m_enabled) {
        m_speed = data;
        m_pid.Compute();
        analogWrite(m_motorPin, m_motorOutput);
    } else {
        analogWrite(m_motorPin, 0);
    }
}

void XV11::Enable(bool enable) {
    m_enabled = enable;
    Update(0);
}
