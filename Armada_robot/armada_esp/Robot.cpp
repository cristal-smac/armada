#include "Robot.h"
#include <Arduino.h>
#include <math.h>

// ─── Private helpers ─────────────────────────────────────────────────────────

float Robot::wheel_speed_to_rpm(float wheel_speed_ms) const
{
    return (wheel_speed_ms / _wheel_circumference) * 60.0f;
}

float Robot::rpm_to_wheel_speed(float rpm) const
{
    return (rpm / 60.0f) * _wheel_circumference;
}

// ─── Public ──────────────────────────────────────────────────────────────────

Robot::Robot(float wheel_diameter,
             float robot_width,
             float max_rpm,
             int left_motor_id,
             int right_motor_id,
             int rx_motors,
             int tx_motors,
             long motors_baudrate,
             bool left_inverted,
             int tof_sda,
             int tof_scl,
             uint8_t tof_mux_address)
    : _left_motor(left_motor_id, rx_motors, tx_motors, motors_baudrate),
      _right_motor(right_motor_id, rx_motors, tx_motors, motors_baudrate),
      _wheel_circumference(M_PI * wheel_diameter),
      _robot_width(robot_width),
      _max_rpm(max_rpm),
      _left_inverted(left_inverted),
      _command_timeout_ms(500),
      _last_command_ms(0),
      _moving(false),
      _timeout_logged(false),
      _lidar(tof_mux_address, tof_sda, tof_scl)
{
}

void Robot::init()
{
    _left_motor.init();
    delay(200);
    _right_motor.init();

    // ─── Branchements du lidar ToF : nom du capteur -> canal du PCA9548A ───
    // L'ordre des add_sensor() donne l'indice de chaque capteur dans le lidar.
    _lidar.add_sensor("F", 0);  // front
    _lidar.add_sensor("FL", 1); // front left
    _lidar.add_sensor("L", 2);  // left
    _lidar.add_sensor("BL", 3); // back left
    _lidar.add_sensor("B", 4);  // back
    _lidar.add_sensor("BR", 5); // back right
    _lidar.add_sensor("R", 6);  // right
    _lidar.add_sensor("FR", 7); // front right
    _lidar.init();
}

bool Robot::set_velocity(float vx, float vw)
{
    float v_left = vx - (vw * _robot_width / 2.0f);
    float v_right = vx + (vw * _robot_width / 2.0f);

    float rpm_left = wheel_speed_to_rpm(v_left);
    float rpm_right = wheel_speed_to_rpm(v_right);

    rpm_left = constrain(rpm_left, -_max_rpm, _max_rpm);
    rpm_right = constrain(rpm_right, -_max_rpm, _max_rpm);

    if (_left_inverted)
    {
        rpm_left = -rpm_left;
    }
    else
    {
        rpm_right = -rpm_right;
    }

    //Serial.printf("[Robot] vx=%.3f m/s  vw=%.3f rad/s → rpm_L=%.2f  rpm_R=%.2f\n", vx, vw, rpm_left, rpm_right);

    bool left_ok = _left_motor.set_rpm_speed(rpm_left);
    delay(20);
    bool right_ok = _right_motor.set_rpm_speed(rpm_right);

    if (!left_ok || !right_ok)
    {
        // Ne jamais rouler sur une seule roue : on coupe les deux moteurs.
        stop();
        return false;
    }

    _last_command_ms = millis();
    _moving = (rpm_left != 0.0f || rpm_right != 0.0f);
    _timeout_logged = false;
    return true;
}

void Robot::get_velocities(float &vx, float &vw)
{
    float rpm_left = _left_motor.get_speed_rpm();
    delay(20);
    float rpm_right = _right_motor.get_speed_rpm();

    if (_left_inverted)
    {
        rpm_left = -rpm_left;
    }
    else
    {
        rpm_right = -rpm_right;
    }

    float v_left = rpm_to_wheel_speed(rpm_left);
    float v_right = rpm_to_wheel_speed(rpm_right);

    vx = (v_left + v_right) / 2.0f;
    vw = (v_right - v_left) / _robot_width;

    // Serial.printf("[Robot] get_velocities → vx=%.3f m/s  vw=%.3f rad/s\n", vx, vw);
}

bool Robot::update_odometry()
{
    float left_rev, right_rev;
    if (!_left_motor.get_position_rev(left_rev))
        return false;
    delay(5);
    if (!_right_motor.get_position_rev(right_rev))
        return false;

    // Même convention de sens que set_velocity() : avancer = positif pour les deux roues.
    if (_left_inverted)
        left_rev = -left_rev;
    else
        right_rev = -right_rev;

    if (!_odom_started)
    {
        // Première lecture : simple référence, pas de déplacement.
        _last_left_rev = left_rev;
        _last_right_rev = right_rev;
        _odom_started = true;
        return true;
    }

    float d_left = (left_rev - _last_left_rev) * _wheel_circumference;
    float d_right = (right_rev - _last_right_rev) * _wheel_circumference;
    _last_left_rev = left_rev;
    _last_right_rev = right_rev;

    float d = (d_left + d_right) / 2.0f;
    float d_theta = (d_right - d_left) / _robot_width;

    // Intégration au cap moyen sur l'intervalle.
    float mid = _odom_theta + d_theta / 2.0f;
    _odom_x += d * cosf(mid);
    _odom_y += d * sinf(mid);
    _odom_theta += d_theta;
    return true;
}

void Robot::get_odometry(float &x, float &y, float &theta) const
{
    x = _odom_x;
    y = _odom_y;
    theta = _odom_theta;
}

void Robot::stop()
{
    bool left_ok = _left_motor.stop();
    bool right_ok = _right_motor.stop();

    // Si un arrêt échoue, on reste "en mouvement" : update() réessaiera au prochain tour.
    _moving = !(left_ok && right_ok);
}

void Robot::set_command_timeout(uint32_t timeout_ms)
{
    _command_timeout_ms = timeout_ms;
}

void Robot::update()
{
    if (!_moving || _command_timeout_ms == 0)
    {
        return;
    }

    if (millis() - _last_command_ms > _command_timeout_ms)
    {
        if (!_timeout_logged)
        {
            Serial.println("[Robot] aucune commande recue, arret des moteurs");
            _timeout_logged = true;
        }
        stop();
    }
}

uint8_t Robot::get_lidar_values(float *values, uint8_t capacity) const
{
    uint8_t n = _lidar.sensor_count();
    if (n > capacity)
    {
        n = capacity;
    }
    for (uint8_t i = 0; i < n; ++i)
    {
        const TofSensor &s = _lidar.sensor(i);
        values[i] = s.valid() ? s.range_mm() / 1000.0f : NAN;
    }
    return n;
}