// Test des 8 capteurs VL53L0X derrière le multiplexeur PCA9548A.
// Même logique que tof.h / tof.cpp (table de capteurs, sélection de canal,
// validité d'une mesure), mais en un seul fichier et sans conversion en points :
// uniquement pour vérifier câblage et mesures.
//
// Câblage : SDA mux -> D21, SCL mux -> D22 (voir description du robot).
// Sortie série (115200) : une ligne par scan, "F=345 FL=-1 ...".
// -1 = mesure invalide (hors portée ou erreur). Compatible Serial Plotter.

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ─── Configuration ──────────────────────────────────────────────────────────
#define SDA_PIN 21 // D21
#define SCL_PIN 22 // D22
#define I2C_CLOCK_HZ 100000

#define PCA9548A_ADDRESS 0x70
#define VL53L0X_ADDRESS 0x29

#define MUX_SETTLE_MS 5      // délai après changement de canal, avant de parler au capteur
#define MAX_RANGE_MM 1200    // au-delà : mesure considérée invalide
#define SCAN_PAUSE_MS 50     // pause entre deux scans complets
#define STATS_EVERY_N_SCANS 10

// Capteurs : nom, canal du multiplexeur, cap dans le repère robot (degrés, anti-horaire).
struct SensorConfig
{
    const char *name;
    uint8_t channel;
    int16_t yaw_deg;
};

const SensorConfig SENSORS[] = {
    {"F", 0, 0},
    {"FL", 1, 45},
    {"L", 2, 90},
    {"BL", 3, 135},
    {"B", 4, 180},
    {"BR", 5, -135},
    {"R", 6, -90},
    {"FR", 7, -45},
};
const uint8_t SENSOR_COUNT = sizeof(SENSORS) / sizeof(SENSORS[0]);

// ─── État ───────────────────────────────────────────────────────────────────
Adafruit_VL53L0X vl53[SENSOR_COUNT];
bool ready[SENSOR_COUNT];        // capteur initialisé avec succès
uint16_t range_mm[SENSOR_COUNT]; // dernière mesure brute
bool valid[SENSOR_COUNT];        // dernière mesure exploitable

uint32_t scan_count = 0;
uint32_t stats_start_ms = 0;

// ─── Utilitaires I2C ────────────────────────────────────────────────────────
bool selectMuxChannel(uint8_t channel)
{
    Wire.beginTransmission(PCA9548A_ADDRESS);
    Wire.write(1 << channel);
    return Wire.endTransmission() == 0;
}

bool i2cPresent(uint8_t address)
{
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

// ─── Setup ──────────────────────────────────────────────────────────────────
void setup()
{
    Serial.begin(115200);
    delay(500);
    Serial.println();
    Serial.println("=== Test 8 capteurs VL53L0X + PCA9548A ===");

    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(I2C_CLOCK_HZ);

    // Sans multiplexeur, rien ne peut fonctionner : on s'arrête ici.
    if (!i2cPresent(PCA9548A_ADDRESS))
    {
        Serial.printf("Multiplexeur PCA9548A introuvable en 0x%02X (SDA=%d, SCL=%d).\n",
                      PCA9548A_ADDRESS, SDA_PIN, SCL_PIN);
        Serial.println("Verifier alimentation, SDA/SCL et adresse A0-A2.");
        while (true)
        {
            delay(1000);
        }
    }
    Serial.printf("Multiplexeur trouve en 0x%02X\n", PCA9548A_ADDRESS);

    uint8_t ok_count = 0;
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
    {
        ready[i] = false;
        range_mm[i] = 0;
        valid[i] = false;

        if (!selectMuxChannel(SENSORS[i].channel))
        {
            Serial.printf("[%-2s ch%u] canal du multiplexeur inaccessible\n",
                          SENSORS[i].name, SENSORS[i].channel);
            continue;
        }
        delay(10); // laisse le mux et le capteur se stabiliser avant la séquence de boot

        if (!i2cPresent(VL53L0X_ADDRESS))
        {
            Serial.printf("[%-2s ch%u] aucun capteur en 0x%02X (cable SDA/SCL/Vin/GND ?)\n",
                          SENSORS[i].name, SENSORS[i].channel, VL53L0X_ADDRESS);
            continue;
        }

        if (!vl53[i].begin(VL53L0X_ADDRESS, false, &Wire))
        {
            Serial.printf("[%-2s ch%u] present mais init echouee\n",
                          SENSORS[i].name, SENSORS[i].channel);
            continue;
        }

        ready[i] = true;
        ++ok_count;
        Serial.printf("[%-2s ch%u] OK (cap %d deg)\n",
                      SENSORS[i].name, SENSORS[i].channel, SENSORS[i].yaw_deg);
    }

    Serial.printf("%u/%u capteurs initialises\n", ok_count, SENSOR_COUNT);
    Serial.println("-------------------------------------------");
    stats_start_ms = millis();
}

// ─── Mesure d'un capteur ────────────────────────────────────────────────────
void readSensor(uint8_t i)
{
    if (!ready[i] || !selectMuxChannel(SENSORS[i].channel))
    {
        valid[i] = false;
        return;
    }
    delay(MUX_SETTLE_MS);

    VL53L0X_RangingMeasurementData_t measure;
    vl53[i].rangingTest(&measure, false); // true = infos de debug

    range_mm[i] = measure.RangeMilliMeter;
    // RangeStatus 4 = phase failure / cible hors de portée.
    valid[i] = measure.RangeStatus != 4 &&
               measure.RangeMilliMeter > 0 &&
               measure.RangeMilliMeter <= MAX_RANGE_MM;
}

// ─── Boucle ─────────────────────────────────────────────────────────────────
void loop()
{
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
    {
        readSensor(i);
    }
    ++scan_count;

    for (uint8_t i = 0; i < SENSOR_COUNT; ++i)
    {
        Serial.print(SENSORS[i].name);
        Serial.print('=');
        Serial.print(valid[i] ? (int)range_mm[i] : -1);
        Serial.print(i + 1 < SENSOR_COUNT ? ' ' : '\n');
    }

    // Durée moyenne d'un scan complet, affichée de temps en temps.
    if (scan_count % STATS_EVERY_N_SCANS == 0)
    {
        uint32_t elapsed = millis() - stats_start_ms;
        Serial.printf("# %u scans en %lu ms : %.0f ms par scan (%.1f Hz)\n",
                      STATS_EVERY_N_SCANS, (unsigned long)elapsed,
                      (float)elapsed / STATS_EVERY_N_SCANS,
                      1000.0f * STATS_EVERY_N_SCANS / elapsed);
        stats_start_ms = millis();
    }

    delay(SCAN_PAUSE_MS);
}
