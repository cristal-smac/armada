#include "Robot.h"

#define WHEEL_DIAMETER 0.0772f //77.2mm // meters - TODO recheck les vrais dimensions
#define ROBOT_WIDTH 0.0925f //92.5mm     // meters - TODO recheck les vrais dimensions
#define MAX_RPM 60.0f

#define LEFT_MOTOR_ID 1
#define RIGHT_MOTOR_ID 2
#define RX_MOTORS 16
#define TX_MOTORS 17
#define MOTORS_BAUDRATE 1000000
#define LEFT_INVERTED  true

Robot robot(WHEEL_DIAMETER,
            ROBOT_WIDTH,
            MAX_RPM,
            LEFT_MOTOR_ID,
            RIGHT_MOTOR_ID,
            RX_MOTORS,
            TX_MOTORS,
            MOTORS_BAUDRATE,
            LEFT_INVERTED);

void setup()
{
    Serial.begin(115200);
    robot.init();
}

void loop()
{
    // 0.05 m/s forward
    robot.set_velocity(0.02f, 0.0f);
    delay(3000);

    robot.stop();
    delay(2000);

    // just rotation of 0.3 rad/s
    robot.set_velocity(0.0f, 0.3f);
    delay(3000);

    robot.stop();
    delay(2000);

     // Read current velocities
    //float read_vx, read_vw;
    //robot.get_velocities(read_vx, read_vw);
}
