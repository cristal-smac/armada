#include "tof.h"
#include <string.h>

// ═══════════════════════════════════════════════════════════════════════════════
//  TofSensor
// ═══════════════════════════════════════════════════════════════════════════════

TofSensor::TofSensor()
    : _name(""),
      _mux_channel(0),
      _max_range_mm(1200),
      _i2c_address(0x29),
      _initialised(false),
      _valid(false),
      _range_mm(0)
{
}

void TofSensor::configure(const char *name,
                          uint8_t mux_channel,
                          uint16_t max_range_mm,
                          uint8_t i2c_address)
{
    _name = name;
    _mux_channel = mux_channel;
    _max_range_mm = max_range_mm;
    _i2c_address = i2c_address;
}

bool TofSensor::init(TwoWire *wire)
{
    // Un capteur absent ou mal câblé ne répond pas du tout : message dédié.
    wire->beginTransmission(_i2c_address);
    if (wire->endTransmission() != 0)
    {
        Serial.printf("[TofSensor %s ch%u] aucun capteur en 0x%02X (cable SDA/SCL/Vin/GND ?)\n",
                      _name, _mux_channel, _i2c_address);
        _initialised = false;
        return false;
    }

    _initialised = _vl53.begin(_i2c_address, false, wire);
    if (!_initialised)
    {
        Serial.printf("[TofSensor %s ch%u] capteur present mais init echouee\n", _name, _mux_channel);
    }
    return _initialised;
}

void TofSensor::update()
{
    if (!_initialised)
    {
        _valid = false;
        return;
    }

    VL53L0X_RangingMeasurementData_t measure;
    _vl53.rangingTest(&measure, false);

    _range_mm = measure.RangeMilliMeter;
    // RangeStatus 4 = phase failure / cible hors de portée.
    _valid = measure.RangeStatus != 4 &&
             measure.RangeMilliMeter > 0 &&
             measure.RangeMilliMeter <= _max_range_mm;
}

void TofSensor::invalidate()
{
    _valid = false;
}

// ═══════════════════════════════════════════════════════════════════════════════
//  TofLidar
// ═══════════════════════════════════════════════════════════════════════════════

TofLidar::TofLidar(uint8_t mux_address,
                   int sda_pin,
                   int scl_pin,
                   uint8_t settle_delay_ms)
    : _count(0),
      _next(0),
      _mux_address(mux_address),
      _sda_pin(sda_pin),
      _scl_pin(scl_pin),
      _settle_delay_ms(settle_delay_ms),
      _wire(&Wire)
{
}

bool TofLidar::select_channel(uint8_t channel)
{
    _wire->beginTransmission(_mux_address);
    _wire->write(1 << channel);
    return _wire->endTransmission() == 0;
}

bool TofLidar::probe(uint8_t address)
{
    _wire->beginTransmission(address);
    return _wire->endTransmission() == 0;
}

bool TofLidar::add_sensor(const char *name, uint8_t mux_channel, uint16_t max_range_mm)
{
    if (_count >= MAX_TOF_SENSORS || mux_channel > 7)
    {
        Serial.printf("[TofLidar] capteur %s refuse (lidar plein ou canal %u invalide)\n", name, mux_channel);
        return false;
    }
    _sensors[_count].configure(name, mux_channel, max_range_mm);
    ++_count;
    return true;
}

bool TofLidar::init()
{
    _wire->begin(_sda_pin, _scl_pin);

    // Sans multiplexeur, aucun capteur ne peut répondre.
    if (!probe(_mux_address))
    {
        Serial.printf("[TofLidar] multiplexeur introuvable en 0x%02X (SDA=%d, SCL=%d)\n",
                      _mux_address, _sda_pin, _scl_pin);
        return false;
    }

    uint8_t ok_count = 0;
    for (uint8_t i = 0; i < _count; ++i)
    {
        if (!select_channel(_sensors[i].channel()))
        {
            Serial.printf("[TofSensor %s ch%u] canal du multiplexeur inaccessible\n",
                          _sensors[i].name(), _sensors[i].channel());
            continue;
        }
        delay(10); // laisse le multiplexeur et le capteur se stabiliser avant la séquence de boot
        if (_sensors[i].init(_wire))
        {
            ++ok_count;
        }
    }

    Serial.printf("[TofLidar] %u/%u capteurs initialises\n", ok_count, _count);
    return ok_count == _count;
}

void TofLidar::update_sensor(uint8_t index)
{
    if (!select_channel(_sensors[index].channel()))
    {
        _sensors[index].invalidate();
        return;
    }
    delay(_settle_delay_ms);
    _sensors[index].update();
}

void TofLidar::update()
{
    for (uint8_t i = 0; i < _count; ++i)
    {
        update_sensor(i);
    }
}

bool TofLidar::update_next()
{
    if (_count == 0)
    {
        return false;
    }

    update_sensor(_next);
    ++_next;
    if (_next >= _count)
    {
        _next = 0;
        return true;
    }
    return false;
}

const TofSensor &TofLidar::sensor(uint8_t index) const
{
    if (index >= _count)
    {
        index = 0;
    }
    return _sensors[index];
}

const TofSensor *TofLidar::find(const char *name) const
{
    for (uint8_t i = 0; i < _count; ++i)
    {
        if (strcmp(_sensors[i].name(), name) == 0)
        {
            return &_sensors[i];
        }
    }
    return nullptr;
}
