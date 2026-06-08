#include "Robot.h"

#define WHEEL_DIAMETER 0.065f // meters - TODO recheck les vrais dimensions
#define ROBOT_WIDTH 0.18f     // meters - TODO recheck les vrais dimensions
#define MAX_RPM 60.0f

#define LEFT_MOTOR_ID 1
#define RIGHT_MOTOR_ID 2
#define RX_MOTORS 16
#define TX_MOTORS 17
#define MOTORS_BAUDRATE 1000000

Robot robot(WHEEL_DIAMETER, ROBOT_WIDTH, MAX_RPM,
            LEFT_MOTOR_ID, RIGHT_MOTOR_ID,
            RX_MOTORS, TX_MOTORS, MOTORS_BAUDRATE);

void setup()
{
    Serial.begin(115200);
    robot.init();
}

void loop()
{
    // 0.1 m/s forward
    robot.set_velocity(0.1f, 0.0f);
    delay(3000);

    // just rotation
    robot.set_velocity(0.0f, 0.5f);
    delay(2000);

    // Read current velocities
    float vx, vw;
    robot.get_velocities(vx, vw);

    robot.stop();
    delay(2000);
}
