// ARMADA : commande du robot par une page web servie par l'ESP32.
//
//  - L'ESP32 crée son propre point d'accès Wi-Fi ("ARMADA-XXXX").
//  - Se connecter à ce réseau, puis ouvrir http://192.168.4.1 : joystick + distances du lidar.
//  - La page appelle /cmd toutes les 100 ms. Sans appel pendant COMMAND_TIMEOUT_MS
//    (déconnexion, page fermée, téléphone verrouillé...), Robot arrête les moteurs.

#include <WiFi.h>
#include <WebServer.h>
#include <esp_mac.h>
#include "Robot.h"
#include "web_page.h"

// ─── Robot ──────────────────────────────────────────────────────────────────
#define WHEEL_DIAMETER 0.0772f // 77.2mm // meters
#define ROBOT_WIDTH 0.0925f    // 92.5mm // meters
#define MAX_RPM 60.0f

#define LEFT_MOTOR_ID 2
#define RIGHT_MOTOR_ID 1
#define RX_MOTORS 16
#define TX_MOTORS 17
#define MOTORS_BAUDRATE 1000000
#define LEFT_INVERTED true

#define TOF_SDA 21
#define TOF_SCL 22
#define TOF_MUX_ADDRESS 0x70

#define FLOOR_TRIG_PIN 27 // HC-SR04 Trig
#define FLOOR_ECHO_PIN 34 // HC-SR04 Echo (5 V -> diviseur de tension)

// ─── Commande ───────────────────────────────────────────────────────────────
#define MAX_LINEAR_SPEED 0.20f  // m/s, joystick à fond vers l'avant
#define MAX_ANGULAR_SPEED 2.0f  // rad/s, joystick à fond sur le côté
#define JOYSTICK_DEADZONE 0.05f // en dessous, l'axe est considéré nul
#define COMMAND_TIMEOUT_MS 400  // sans commande pendant ce délai, le robot s'arrête

// ─── Wi-Fi (point d'accès) ──────────────────────────────────────────────────
#define AP_SSID_PREFIX "ARMADA"
#define AP_PASSWORD "armada1234" // 8 caractères minimum (WPA2) : à changer
#define AP_CHANNEL 1
#define AP_MAX_CLIENTS 2
#define WEB_PORT 80

Robot robot(WHEEL_DIAMETER,
            ROBOT_WIDTH,
            MAX_RPM,
            LEFT_MOTOR_ID,
            RIGHT_MOTOR_ID,
            RX_MOTORS,
            TX_MOTORS,
            MOTORS_BAUDRATE,
            LEFT_INVERTED,
            TOF_SDA,
            TOF_SCL,
            TOF_MUX_ADDRESS,
            FLOOR_TRIG_PIN,
            FLOOR_ECHO_PIN);

WebServer server(WEB_PORT);

float cmd_vx = 0.0f;    // dernière consigne reçue (m/s)
float cmd_vw = 0.0f;    // dernière consigne reçue (rad/s)
bool robot_idle = true; // true : moteurs arrêtés

// ─── Serveur web ────────────────────────────────────────────────────────────

// Lit joystick (-1..1). Argument absent => 0 (stop).
float read_axis(const char *name)
{
    float v = server.arg(name).toFloat();
    if (isnan(v))
    {
        return 0.0f;
    }
    v = constrain(v, -1.0f, 1.0f);
    return fabsf(v) < JOYSTICK_DEADZONE ? 0.0f : v;
}

// Réponse JSON : consigne en cours + distances du lidar (null = mesure invalide)
void send_state()
{
    float values[MAX_TOF_SENSORS];
    uint8_t n = robot.get_lidar_values(values, MAX_TOF_SENSORS);

    String json = "{\"vx\":" + String(cmd_vx, 3) + ",\"vw\":" + String(cmd_vw, 3) + ",\"lidar\":[";
    for (uint8_t i = 0; i < n; ++i)
    {
        if (i > 0)
        {
            json += ',';
        }
        json += "{\"n\":\"";
        json += robot.lidar().sensor(i).name();
        json += "\",\"d\":";
        json += isnan(values[i]) ? String("null") : String(values[i], 3);
        json += '}';
    }
    float floor_m = robot.get_floor_distance();
    json += "],\"floor\":" + (isnan(floor_m) ? String("null") : String(floor_m, 3)) + "}";

    server.sendHeader("Cache-Control", "no-store");
    server.send(200, "application/json", json);
}

void handle_root()
{
    server.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

// /cmd?x=<-1..1>&y=<-1..1> : x = droite (+), y = avant (+)
void handle_cmd()
{
    float x = read_axis("x");
    float y = read_axis("y");

    cmd_vx = y * MAX_LINEAR_SPEED;
    cmd_vw = -x * MAX_ANGULAR_SPEED; // x positif = vers la droite = rotation horaire = vw négatif

    // Consigne nulle déjà envoyée et confirmée
    bool idle = (cmd_vx == 0.0f && cmd_vw == 0.0f);
    if (!(idle && robot_idle))
    {
        bool ok = robot.set_velocity(cmd_vx, cmd_vw);
        robot_idle = idle && ok;
    }
    send_state();
}

void handle_stop()
{
    cmd_vx = 0.0f;
    cmd_vw = 0.0f;
    robot.stop();
    robot_idle = true;
    send_state();
}

void start_access_point()
{
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char ssid[24];
    snprintf(ssid, sizeof(ssid), AP_SSID_PREFIX "-%02X%02X", mac[4], mac[5]); // un nom par robot

    WiFi.mode(WIFI_AP);
    if (!WiFi.softAP(ssid, AP_PASSWORD, AP_CHANNEL, 0, AP_MAX_CLIENTS))
    {
        Serial.println("[Wi-Fi] echec du point d'acces");
        return;
    }
    Serial.printf("[Wi-Fi] reseau \"%s\", mot de passe \"%s\"\n", ssid, AP_PASSWORD);
    Serial.printf("[Wi-Fi] page de commande : http://%s\n", WiFi.softAPIP().toString().c_str());
}

// ─── Arduino ────────────────────────────────────────────────────────────────

void setup()
{
    Serial.begin(115200);

    robot.init();
    robot.set_command_timeout(COMMAND_TIMEOUT_MS);

    start_access_point();
    server.on("/", HTTP_GET, handle_root);
    server.on("/cmd", HTTP_GET, handle_cmd);
    server.on("/stop", HTTP_GET, handle_stop);
    server.onNotFound([]()
                      { server.send(404, "text/plain", "404"); });
    server.begin();
}

void loop()
{
    server.handleClient();       // reçoit les commandes de la page
    robot.update();              // arret si plus de commande
    robot.floor_sensor().update(); // HC-SR04 : une mesure toutes les 60 ms
    robot.lidar().update_next(); // un capteur ToF par tour (environ 46 ms en mesurant pendant les tests)
}
