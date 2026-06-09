#include "Motor.h"
// ─── Private helpers ─────────────────────────────────────────────────────────

int Motor::rpm_to_ticks(float rpm) const
{
    return static_cast<int>(rpm / 60.0f * TICKS_PER_REV);
}

float Motor::ticks_to_rpm(int ticks) const
{
    return static_cast<float>(ticks) * 60.0f / TICKS_PER_REV;
}

// ─── Public ──────────────────────────────────────────────────────────────────

Motor::Motor(int id, int rxd, int txd, long baudrate)
    : _serial(1),
      _id(id),
      _rxd(rxd),
      _txd(txd),
      _baudrate(baudrate)
{
    _serial.begin(_baudrate, SERIAL_8N1, _rxd, _txd);
    _sms_sts.pSerial = &_serial;
    delay(1000);
}

bool Motor::init()
{
    int result = _sms_sts.WritePosEx(_id, 0, 0, 0);
    if (result == -1)
    {
        Serial.printf("[Motor %d] Erreur init\n", _id);
        return false;
    }
    return true;
}

bool Motor::set_rpm_speed(float rpm)
{
    if (rpm > MAX_RPM)
        rpm = MAX_RPM;

    int ticks = rpm_to_ticks(rpm);
    // Serial.printf("[Motor %d] set_rpm_speed(%.2f RPM) → %d ticks/s\n", _id, rpm, ticks);

    int result = _sms_sts.WritePosEx(_id, 4095, ticks, 20);
    if (result == -1)
    {
        Serial.printf("[Motor %d] Erreur : impossible de régler la vitesse\n", _id);
        return false;
    }
    return true;
}

float Motor::get_speed_rpm()
{
    int ticks = _sms_sts.ReadSpeed(_id);
    if (ticks == -1)
    {
        Serial.printf("[Motor %d] Erreur : impossible de lire la vitesse\n", _id);
        return -1.0f;
    }
    float rpm = ticks_to_rpm(ticks);
    // Serial.printf("[Motor %d] get_speed_rpm() → %d ticks/s = %.2f RPM\n", _id, ticks, rpm);
    return rpm;
}

bool Motor::stop()
{
    int result = _sms_sts.WritePosEx(_id, 0, 0, 20);
    if (result == -1)
    {
        Serial.printf("[Motor %d] Erreur : impossible d'arrêter le moteur\n", _id);
        return false;
    }
    return true;
}