#pragma once
#include <SCServo.h>

class Motor
{
public:
    Motor(int id, int rxd = 16, int txd = 17, long baudrate = 1000000); // STS3215 servo spec 1M baudrate

    bool init();
    bool set_rpm_speed(float rpm);
    float get_speed_rpm();
    bool stop();

private:
    static constexpr float TICKS_PER_REV = 4095.0f; // STS3215 servo spec 4095 tic for 1 revolution
    static constexpr float MAX_RPM = 60.0f;
    static constexpr int SPEED_ATTEMPTS = 2; // essais pour une consigne de vitesse
    static constexpr int STOP_ATTEMPTS = 3;  // essais pour un arrêt (plus important)

    int rpm_to_ticks(float rpm) const;
    float ticks_to_rpm(int ticks) const;

    SMS_STS _sms_sts;
    HardwareSerial _serial;
    int _id;
    int _rxd;
    int _txd;
    long _baudrate;
};