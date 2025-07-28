#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// pins I2C pour l'ESP32
#define SDA_PIN 2 // D2
#define SCL_PIN 4 // D4

// Adresse I2C du multiplexeur PCA9548A
#define PCA9548A_ADDRESS 0x70

// Canal du multiplexeur auquel le VL53L0X est connecté
#define SENSOR_CHANNEL 2

// Adresse I2C par défaut du VL53L0X
#define VL53L0X_ADDRESS 0x29

// Initialisation du capteur VL53L0X
Adafruit_VL53L0X vl53 = Adafruit_VL53L0X();

void setup()
{
    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);

    // Sélectionner le canal du multiplexeur
    selectMuxChannel(SENSOR_CHANNEL);
    delay(100); // Ajouter un petit délai

    // Initialiser le capteur VL53L0X
    if (!vl53.begin(VL53L0X_I2C_ADDR, &Wire))
    {
        Serial.println("Échec de l'initialisation du VL53L0X");
        while (1)
            ;
    }
    Serial.println("VL53L0X initialisé");
}

void loop()
{
    VL53L0X_RangingMeasurementData_t measure;

    // lecture distance
    vl53.rangingTest(&measure, false); // true pour mesures de debug

    if (measure.RangeStatus != 4)
    { // Phase failures have a value of 4
        Serial.print("Distance (mm): ");
        Serial.println(measure.RangeMilliMeter);
    }
    else
    {
        Serial.println("Échec de la mesure");
    }

    delay(1000);
}

void selectMuxChannel(uint8_t channel)
{
    Wire.beginTransmission(PCA9548A_ADDRESS);
    Wire.write(1 << channel);
    Wire.endTransmission();
}
