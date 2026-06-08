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
             bool left_inverted)
    : _left_motor(left_motor_id, rx_motors, tx_motors, motors_baudrate),
      _right_motor(right_motor_id, rx_motors, tx_motors, motors_baudrate),
      _wheel_circumference(M_PI * wheel_diameter),
      _robot_width(robot_width),
      _max_rpm(max_rpm),
      _left_inverted(left_inverted)
{
}

void Robot::init()
{
    _left_motor.init();
    _right_motor.init();
}

void Robot::set_velocity(float vx, float vw)
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
        rpm_right = -rpm_right
    };

    // Serial.printf("[Robot] vx=%.3f m/s  vw=%.3f rad/s → rpm_L=%.2f  rpm_R=%.2f\n", vx, vw, rpm_left, rpm_right);

    _left_motor.set_rpm_speed(rpm_left);
    _right_motor.set_rpm_speed(rpm_right);
}

void Robot::get_velocities(float &vx, float &vw)
{
    float rpm_left = _left_motor.get_speed_rpm();
    float rpm_right = _right_motor.get_speed_rpm();

    if (_left_inverted)
    {
        rpm_left = -rpm_left;
    }
    else
    {
        rpm_right = -rpm_right
    };

    float v_left = rpm_to_wheel_speed(rpm_left);
    float v_right = rpm_to_wheel_speed(rpm_right);

    vx = (v_left + v_right) / 2.0f;
    vw = (v_right - v_left) / _robot_width;

    // Serial.printf("[Robot] get_velocities → vx=%.3f m/s  vw=%.3f rad/s\n", vx, vw);
}

void Robot::stop()
{
    _left_motor.stop();
    _right_motor.stop();
}