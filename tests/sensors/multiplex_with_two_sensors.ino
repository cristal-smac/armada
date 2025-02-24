#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// Adresse I2C du multiplexeur PCA9548A
#define PCA9548A_ADDRESS 0x70

// Canal du multiplexeur auquel le VL53L0X est connecté
#define SENSOR1_CHANNEL 2

#define SENSOR2_CHANNEL 3

// Adresse I2C par défaut du VL53L0X
#define VL53L0X_ADDRESS 0x29

// Initialisation du capteur VL53L0X
Adafruit_VL53L0X vl53_1 = Adafruit_VL53L0X();

Adafruit_VL53L0X vl53_2 = Adafruit_VL53L0X();

// pins I2C pour l'ESP32
#define SDA_PIN 2 // D2
#define SCL_PIN 4 // D4

void setup()
{
    Serial.begin(115200);

    Wire.begin(SDA_PIN, SCL_PIN);

    // TODO : recoder en fonction, pas en dur degueulasse.
    // TODO : faire un tableau d'objet capteurs, ce sera mieux je pense
    //  Sélectionner le canal du multiplexeur
    selectMuxChannel(SENSOR1_CHANNEL);
    delay(10); // Ajouter un petit délai

    // Initialiser le capteur VL53L0X
    if (!vl53_1.begin(VL53L0X_I2C_ADDR, &Wire))
    {
        Serial.println("Échec de l'initialisation du VL53L0X 1");
        while (1)
            ;
    }
    Serial.println("VL53L0X initialisé");

    //-------------------------------------------------------------
    // Sélectionner le canal du multiplexeur
    selectMuxChannel(SENSOR2_CHANNEL);
    delay(10); // Ajouter un petit délai

    // Initialiser le capteur VL53L0X
    if (!vl53_2.begin(VL53L0X_I2C_ADDR, &Wire))
    {
        Serial.println("Échec de l'initialisation du VL53L0X 2");
        while (1)
            ;
    }
    Serial.println("VL53L0X initialisé");
    //------------------------------------------------------------
}

void loop()
{
    VL53L0X_RangingMeasurementData_t measure1;
    VL53L0X_RangingMeasurementData_t measure2;

    // Lire la distance
    // Sélectionner le canal du multiplexeur
    selectMuxChannel(SENSOR1_CHANNEL);
    delay(10);                            // Ajouter un petit délai
    vl53_1.rangingTest(&measure1, false); // passer à true pour obtenir des mesures de debug

    // Sélectionner le canal du multiplexeur
    selectMuxChannel(SENSOR2_CHANNEL);
    delay(10); // Ajouter un petit délai
    vl53_2.rangingTest(&measure2, false);

    if (measure1.RangeStatus != 4)
    { // Phase failures have a value of 4
        Serial.print("Distance (mm): ");
        Serial.println(measure1.RangeMilliMeter);
    }
    else
    {
        Serial.println("Échec de la mesure");
    }
    if (measure2.RangeStatus != 4)
    { // Phase failures have a value of 4
        Serial.print("Distance (mm): ");
        Serial.println(measure2.RangeMilliMeter);
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
