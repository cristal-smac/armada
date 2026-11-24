#include "ultrasonic.h"

UltrasonicSensor::UltrasonicSensor(int trig_pin, int echo_pin, uint16_t max_range_mm, uint16_t period_ms)
    : _trig(trig_pin),
      _echo(echo_pin),
      _max_range_mm(max_range_mm),
      _period_ms(period_ms)
{
}

void UltrasonicSensor::init()
{
    pinMode(_trig, OUTPUT);
    digitalWrite(_trig, LOW);
    pinMode(_echo, INPUT);
}

void UltrasonicSensor::update()
{
    uint32_t now = millis();
    if (now - _last_ms < _period_ms)
    {
        return;
    }
    _last_ms = now;

    // Impulsion de 10 µs sur Trig
    digitalWrite(_trig, LOW);
    delayMicroseconds(2);
    digitalWrite(_trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trig, LOW);

    // Aller-retour du son : 2 * distance / 343 m/s, soit ~5,8 µs par mm de distance.
    uint32_t timeout_us = (uint32_t)_max_range_mm * 6 + 500;
    uint32_t pulse_us = pulseIn(_echo, HIGH, timeout_us);

    if (pulse_us == 0)
    {
        _valid = false; // pas d'écho : hors portée ou capteur absent (via datasheet)
        return;
    }

    uint32_t mm = pulse_us * 343UL / 2000UL; // µs * (343 mm/ms) / 2 / 1000
    _range_mm = (uint16_t)mm;
    _valid = (mm <= _max_range_mm);
}
