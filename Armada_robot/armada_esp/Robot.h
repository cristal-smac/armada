#pragma once
#include "Motor.h"
#include "tof.h"

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
          uint8_t tof_mux_address = 0x70);

    /** Initialise les moteurs et le lidar ToF. À appeler une seule fois, dans setup(). */
    void init();

    /**
     * diff drive speed.
     * @param vx  linear speed in m/s
     * @param vw  angular speed in rad/s
     */
    void set_velocity(float vx, float vw);

    /**
     * diff drive speed.
     * @param vx  linear speed in m/s
     * @param vw  angular speed in rad/s
     */
    void get_velocities(float &vx, float &vw);

    /** stops the motors */
    void stop();

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

    TofLidar _lidar;
};
