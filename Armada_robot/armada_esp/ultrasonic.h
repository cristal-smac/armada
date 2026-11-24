#pragma once
#include <Arduino.h>

// ─────────────────────────────────────────────────────────────────────────────
//  UltrasonicSensor : un HC-SR04 (trigger + echo).
// ─────────────────────────────────────────────────────────────────────────────
class UltrasonicSensor
{
public:
    /**
     * @param trig_pin      GPIO de sortie vers Trig
     * @param echo_pin      GPIO d'entrée depuis Echo (34 : entrée seule, convient)
     * @param max_range_mm  au-delà de cette distance la mesure est invalide
     * @param period_ms     délai minimal entre deux mesures réelles de update()
     */
    UltrasonicSensor(int trig_pin, int echo_pin, uint16_t max_range_mm = 400, uint16_t period_ms = 60);

    /** Configure les broches. À appeler une fois, dans setup(). */
    void init();

    /**
     * Fait une mesure si period_ms s'est écoulé depuis la dernière (sinon ne fait rien).
     * Bloque au plus le temps d'aller-retour du son sur max_range_mm (~2 ms à 40 cm).
     */
    void update();

    /** true si la dernière mesure est exploitable. */
    bool valid() const { return _valid; }

    /** Dernière distance en mm, ou -1 si la mesure est invalide. */
    int16_t distance_mm() const { return _valid ? (int16_t)_range_mm : -1; }

private:
    int _trig;
    int _echo;
    uint16_t _max_range_mm;
    uint16_t _period_ms;
    uint32_t _last_ms = 0;
    uint16_t _range_mm = 0;
    bool _valid = false;
};
