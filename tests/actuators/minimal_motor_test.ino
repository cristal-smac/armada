// Motor control pins
const int motorPin1 = 26; // IN1 on the motor driver
const int motorPin2 = 27; // IN2 on the motor driver
const int enablePin = 25; // ENA on the motor driver

void setup()
{
    // Set motor control pins as outputs
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(enablePin, OUTPUT);
}

void loop()
{
    // Set motor direction to forward
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);

    // Ramp up motor speed
    for (int speed = 0; speed <= 255; speed++)
    {
        analogWrite(enablePin, speed);
        delay(15); // Small delay to observe the speed change
    }

    // Stop the motor
    analogWrite(enablePin, 0);
    delay(2000); // Wait for 2 seconds

    // Set motor direction to backward
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);

    // Ramp up motor speed in reverse
    for (int speed = 0; speed <= 255; speed++)
    {
        analogWrite(enablePin, speed);
        delay(15); // Small delay to observe the speed change
    }

    // Stop the motor
    analogWrite(enablePin, 0);
    delay(2000); // Wait for 2 seconds
}