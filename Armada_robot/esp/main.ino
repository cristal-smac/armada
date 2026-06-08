#include "Motor.h"

// motors_vars
#define LEFT_MOTOR_ID 1
#define RIGHT_MOTOR_ID 2
#define RX_MOTORS 16
#define TX_MOTORS 17
#define MOTORS_BAUDRATE 1000000

Motor leftmotor(LEFT_MOTOR_ID, RX_MOTORS, TX_MOTORS, MOTORS_BAUDRATE);
Motor rightmotor(RIGHT_MOTOR_ID, RX_MOTORS, TX_MOTORS, MOTORS_BAUDRATE);

void setup()
{
    Serial.begin(115200);
    leftmotor.init();
    rightmotor.init();
}

void loop() // TODO : faire une classe robot pour mettre des vitesses en cmd vel
{
    leftmotor.set_rpm_speed(30.0f);  // 30 RPM -> 1 roll every 2 secs
    rightmotor.set_rpm_speed(30.0f); // 30 RPM -> 1 roll every 2 secs
    delay(3000);
    leftmotor.stop();
    rightmotor.stop();
    delay(2000);
}