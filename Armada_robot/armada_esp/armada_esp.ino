#include "Robot.h"

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
            TOF_MUX_ADDRESS);

// Affiche une ligne "F=0.059 FL=0.220 ..." (distances en m, "nan" = mesure invalide)
void print_lidar()
{
    float values[MAX_TOF_SENSORS];
    uint8_t n = robot.get_lidar_values(values, MAX_TOF_SENSORS);
    for (uint8_t i = 0; i < n; ++i)
    {
        Serial.print(robot.lidar().sensor(i).name());
        Serial.print('=');
        Serial.print(values[i], 3);
        Serial.print(i + 1 < n ? ' ' : '\n');
    }
}

// Attend duration_ms en mesurant le lidar (un capteur par appel) et affiche chaque scan complet.
void wait_with_lidar(uint32_t duration_ms)
{
    uint32_t start = millis();
    while (millis() - start < duration_ms)
    {
        if (robot.lidar().update_next())
        {
            print_lidar();
        }
    }
}

void setup()
{
    Serial.begin(115200);
    robot.init();
}

void loop()
{
    // 0.06 m/s forward
    robot.set_velocity(0.06f, 0.0f);
    wait_with_lidar(3000);

    robot.stop();
    wait_with_lidar(2000);

    // // just rotation of 0.3 rad/s
    // robot.set_velocity(0.0f, 0.3f);
    // wait_with_lidar(3000);

    // robot.stop();
    // wait_with_lidar(2000);

    // Read current velocities
    // float read_vx, read_vw;
    // robot.get_velocities(read_vx, read_vw);
}
