#pragma once
#include "Motor.h"
#include "tof.h"
#include "ultrasonic.h"

class Robot
{
public:
    /**
     * @param wheel_diameter
     * @param robot_width distance between the 2 wheels in meters
     * @param max_rpm max rpm speed on each motors
     * @param left_motor_id
     * @param right_motor_id
     * @param rx_motors
     * @param tx_motors
     * @param motors_baudrate
     * @param left_inverted left_motor_inverted? (false=right_inverted)
     * @param tof_sda I2C SDA pin of the ToF multiplexer
     * @param tof_scl I2C SCL pin of the ToF multiplexer
     * @param tof_mux_address I2C address of the PCA9548A
     * @param floor_trig_pin  Trig pin of the HC-SR04 (floor sensor)
     * @param floor_echo_pin  Echo pin of the HC-SR04 (5 V -> voltage divider needed)
     */
    Robot(float wheel_diameter,
          float robot_width,
          float max_rpm = 60.0f,
          int left_motor_id = 1,
          int right_motor_id = 2,
          int rx_motors = 16,
          int tx_motors = 17,
          long motors_baudrate = 1000000,
          bool left_inverted = true,
          int tof_sda = 21,
          int tof_scl = 22,
          uint8_t tof_mux_address = 0x70,
          int floor_trig_pin = 27,
          int floor_echo_pin = 34);

    /** Initialise les moteurs et le lidar ToF. À appeler une seule fois, dans setup(). */
    void init();

    /**
     * diff drive speed.
     * @param vx  linear speed in m/s
     * @param vw  angular speed in rad/s
     * @return false si un moteur n'a pas répondu : les deux moteurs sont alors arrêtés
     *         (le robot ne roule jamais sur une seule roue).
     *
     * Chaque appel relance le délai de sécurité (voir set_command_timeout).
     */
    bool set_velocity(float vx, float vw);

    /**
     * diff drive speed.
     * @param vx  linear speed in m/s
     * @param vw  angular speed in rad/s
     */
    void get_velocities(float &vx, float &vw);

    /**
     * Odométrie : intègre le déplacement des roues depuis le dernier appel.
     * À appeler régulièrement (au moins plusieurs fois par tour de roue), ex. dans loop().
     * @return false si un moteur n'a pas répondu (rien n'est mis à jour)
     */
    bool update_odometry();

    /**
     * Pose estimée depuis le démarrage (repère de départ du robot).
     * @param x      position en m (avant = +x au départ)
     * @param y      position en m
     * @param theta  cap en rad (anti-horaire positif)
     */
    void get_odometry(float &x, float &y, float &theta) const;

    /** stops the motors */
    void stop();

    /**
     * Sécurité : si le robot roule et qu'aucun set_velocity() n'arrive pendant
     * @p timeout_ms, update() arrête les moteurs (déconnexion, plantage de l'interface...).
     * 0 désactive la sécurité. Valeur par défaut : 500 ms.
     */
    void set_command_timeout(uint32_t timeout_ms);

    /**
     * À appeler à chaque tour de loop(), même sans nouvelle commande :
     * applique la sécurité ci-dessus.
     */
    void update();

    /**
     * Lidar ToF du robot : indique quel capteur voit quelle distance.
     * Exemple : robot.lidar().update_next(); robot.lidar().find("F")->distance_mm();
     */
    TofLidar &lidar() { return _lidar; }

    /**
     * Dernières distances du lidar, sans refaire de mesure (voir lidar().update_next()).
     * @param values    tableau de sortie, une distance par capteur dans l'ordre du lidar
     *                  (même ordre que robot.lidar().sensor(i))
     * @param capacity  taille de @p values
     * @return nombre de valeurs écrites
     *
     * Distance en mètres, ou NAN si la mesure du capteur est invalide
     * (hors portée, erreur, capteur absent). Tester avec isnan(values[i]).
     */
    uint8_t get_lidar_values(float *values, uint8_t capacity) const;

    /**
     * Capteur à ultrasons HC-SR04 (incliné vers le sol, détection de vide).
     * Exemple : robot.floor_sensor().update(); (à appeler dans loop(), limite sa cadence seul)
     */
    UltrasonicSensor &floor_sensor() { return _floor_sensor; }

    /**
     * Dernière distance du HC-SR04 en mètres, ou NAN si la mesure est invalide
     * (pas d'écho, hors portée). Tester avec isnan().
     */
    float get_floor_distance() const;

private:
    /** linear speed (m/s) of a wheel in RPM. */
    float wheel_speed_to_rpm(float wheel_speed_ms) const;

    /** rpm speed to linear (m/s) for a wheel. */
    float rpm_to_wheel_speed(float rpm) const;

    Motor _left_motor;
    Motor _right_motor;

    float _wheel_circumference; // pi * wheel_diameter (in m)
    float _robot_width;         // distance between wheels (in m)
    float _max_rpm;
    bool _left_inverted;

    uint32_t _command_timeout_ms; // 0 = sécurité désactivée
    uint32_t _last_command_ms;    // instant du dernier set_velocity() réussi
    bool _moving;                 // true si une consigne non nulle est active (ou si un arrêt a échoué)
    bool _timeout_logged;         // évite de répéter le message de sécurité

    // Odométrie
    float _odom_x = 0.0f;
    float _odom_y = 0.0f;
    float _odom_theta = 0.0f;
    float _last_left_rev = 0.0f;
    float _last_right_rev = 0.0f;
    bool _odom_started = false;

    TofLidar _lidar;
    UltrasonicSensor _floor_sensor;
};
