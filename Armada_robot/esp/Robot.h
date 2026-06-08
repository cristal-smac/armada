#pragma once
#include "Motor.h"

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
     */
    Robot(float wheel_diameter,
          float robot_width,
          float max_rpm = 60.0f,
          int left_motor_id = 1,
          int right_motor_id = 2,
          int rx_motors = 16,
          int tx_motors = 17,
          long motors_baudrate = 1000000,
          bool left_inverted = true);

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
};