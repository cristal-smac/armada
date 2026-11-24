#include "Motor.h"
// ─── Private helpers ─────────────────────────────────────────────────────────

int Motor::rpm_to_ticks(float rpm) const
{
    return static_cast<int>((rpm / 60.0f) * TICKS_PER_REV);
}

float Motor::ticks_to_rpm(int ticks) const
{
    return static_cast<float>((ticks * 60.0f) / TICKS_PER_REV);
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
    // SCServo : les commandes d'écriture renvoient 1 si le servo a répondu, 0 sinon.
    int result = _sms_sts.WritePosEx(_id, 0, 0, 0);
    if (result != 1)
    {
        Serial.printf("[Motor %d] Erreur init : pas de reponse du servo\n", _id);
        return false;
    }
    _sms_sts.EnableTorque(_id, 1);
    delay(500);
    _sms_sts.WheelMode(_id);
    delay(500);

    return true;
}

bool Motor::set_rpm_speed(float rpm)
{
    rpm = constrain(rpm, -MAX_RPM, MAX_RPM);

    int ticks = rpm_to_ticks(rpm);
    // Serial.printf("[Motor %d] set_rpm_speed(%.2f RPM) → %d ticks/s\n", _id, rpm, ticks);

    // Nombre d'essais limite pour pas bloquer le firmware
    for (int attempt = 0; attempt < SPEED_ATTEMPTS; ++attempt)
    {
        if (_sms_sts.WriteSpe(_id, ticks, 20) == 1)
        {
            return true;
        }
    }
    Serial.printf("[Motor %d] Erreur : pas de reponse du servo (vitesse)\n", _id);
    return false;
}

float Motor::get_speed_rpm()
{
    int ticks = _sms_sts.ReadSpeed(_id);
    if (ticks == -1)
    {
        // Serial.printf("[Motor %d] Erreur : impossible de lire la vitesse\n", _id);
        return -1.0f;
    }
    float rpm = ticks_to_rpm(ticks);
    // Serial.printf("[Motor %d] get_speed_rpm() → %d ticks/s = %.2f RPM\n", _id, ticks, rpm);
    return rpm;
}

bool Motor::get_position_rev(float &rev)
{
    int pos = _sms_sts.ReadPos(_id);
    if (pos == -1)
    {
        return false;
    }
    if (_last_pos != -1)
    {
        int delta = pos - _last_pos;
        // Passage par 0 / 4095 : on prend le chemin le plus court.
        if (delta > POS_RANGE / 2)
            delta -= POS_RANGE;
        else if (delta < -POS_RANGE / 2)
            delta += POS_RANGE;
        _total_ticks += delta;
    }
    _last_pos = pos;
    rev = static_cast<float>(_total_ticks) / POS_RANGE;
    return true;
}

bool Motor::stop()
{
    for (int attempt = 0; attempt < STOP_ATTEMPTS; ++attempt)
    {
        if (_sms_sts.WritePosEx(_id, 0, 0, 20) == 1)
        {
            return true;
        }
    }
    Serial.printf("[Motor %d] Erreur : impossible d'arreter le moteur\n", _id);
    return false;
}