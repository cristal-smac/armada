#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Nombre maximum de capteurs ToF qu'un TofLidar peut piloter
// (le multiplexeur PCA9548A expose 8 canaux).
#define MAX_TOF_SENSORS 8

// ─────────────────────────────────────────────────────────────────────────────
//  TofSensor : un VL53L0X derrière un canal du multiplexeur I2C.
//  Il connaît son nom (F, FL, ...), son canal et sa dernière mesure.
//  Il ne gère PAS le multiplexeur : c'est TofLidar qui sélectionne le canal
//  avant d'appeler init() ou update().
// ─────────────────────────────────────────────────────────────────────────────
class TofSensor
{
public:
    TofSensor();

    /**
     * @param name          nom du capteur ("F", "FL", ...). Doit être une chaîne littérale
     *                      (le pointeur est conservé, pas la chaîne).
     * @param mux_channel   canal PCA9548A (0..7) auquel le capteur est branché
     * @param max_range_mm  au-delà de cette distance la mesure est invalide
     * @param i2c_address   adresse I2C du VL53L0X (0x29 par défaut : le multiplexeur
     *                      isole les bus, pas besoin de la changer)
     */
    void configure(const char *name,
                   uint8_t mux_channel,
                   uint16_t max_range_mm = 1200,
                   uint8_t i2c_address = 0x29);

    /** Initialise le VL53L0X. Le canal du multiplexeur doit déjà être sélectionné. */
    bool init(TwoWire *wire = &Wire);

    /** Fait une mesure. Le canal du multiplexeur doit déjà être sélectionné. */
    void update();

    /** Marque la dernière mesure comme invalide (ex. multiplexeur injoignable). */
    void invalidate();

    const char *name() const { return _name; }
    uint8_t channel() const { return _mux_channel; }
    bool ready() const { return _initialised; }

    /** true si la dernière mesure est exploitable. */
    bool valid() const { return _valid; }

    /** Dernière distance brute en mm (même si invalide). */
    uint16_t range_mm() const { return _range_mm; }

    /** Dernière distance en mm, ou -1 si la mesure est invalide. */
    int16_t distance_mm() const { return _valid ? (int16_t)_range_mm : -1; }

private:
    Adafruit_VL53L0X _vl53;
    const char *_name;
    uint8_t _mux_channel;
    uint16_t _max_range_mm;
    uint8_t _i2c_address;
    bool _initialised;
    bool _valid;
    uint16_t _range_mm;
};

// ─────────────────────────────────────────────────────────────────────────────
//  TofLidar : possède le multiplexeur et les TofSensor. Il indique simplement
//  quel capteur voit quelle distance, sans position ni géométrie.
//
//  Utilisation (en général via Robot, qui s'en charge) :
//    TofLidar lidar;                  // ne touche pas au matériel
//    lidar.add_sensor("F", 0);        // nom + canal du multiplexeur
//    lidar.add_sensor("FL", 1);
//    lidar.init();                    // démarre l'I2C et initialise les capteurs
//    lidar.update();                  // lit tous les capteurs (bloquant)
//    lidar.find("F")->distance_mm();  // distance du capteur avant, -1 si invalide
// ─────────────────────────────────────────────────────────────────────────────
class TofLidar
{
public:
    /**
     * @param mux_address     adresse I2C du PCA9548A
     * @param sda_pin         broche SDA de l'ESP32
     * @param scl_pin         broche SCL de l'ESP32
     * @param settle_delay_ms délai après changement de canal avant de parler au capteur
     */
    TofLidar(uint8_t mux_address = 0x70,
             int sda_pin = 21,
             int scl_pin = 22,
             uint8_t settle_delay_ms = 5);

    /** Déclare un capteur. À appeler avant init(). Retourne false si le lidar est plein. */
    bool add_sensor(const char *name, uint8_t mux_channel, uint16_t max_range_mm = 1200);

    /** Démarre l'I2C et initialise chaque capteur. Retourne true si tous répondent. */
    bool init();

    /** Mesure tous les capteurs (bloquant, environ 46 ms par capteur). */
    void update();

    /**
     * Mesure un seul capteur, puis passe au suivant au prochain appel.
     * Permet de ne pas bloquer la boucle principale pendant un scan complet.
     * @return true quand un cycle complet vient de se terminer
     */
    bool update_next();

    uint8_t sensor_count() const { return _count; }

    /** Capteur d'indice @p index (ordre des add_sensor). Indice invalide : premier capteur. */
    const TofSensor &sensor(uint8_t index) const;

    /** Capteur de nom @p name, ou nullptr s'il n'existe pas. */
    const TofSensor *find(const char *name) const;

private:
    bool select_channel(uint8_t channel);
    bool probe(uint8_t address);
    void update_sensor(uint8_t index);

    TofSensor _sensors[MAX_TOF_SENSORS];
    uint8_t _count;
    uint8_t _next; // prochain capteur pour update_next()
    uint8_t _mux_address;
    int _sda_pin;
    int _scl_pin;
    uint8_t _settle_delay_ms;
    TwoWire *_wire;
};
