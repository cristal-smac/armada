// Motor control pins
const int motorPin1 = 2;  // IN3 on the motor driver
const int motorPin2 = 15; // IN4 on the motor driver
const int enablePin = 18; // ENB on the motor driver

// pwm parameters setup
const int freq = 30000;
const int pwmChannelL = 1;
const int resolution = 8;

void SetMotorSpeed(int direction, int speed, int pin1, int pin2, int enpin)
{
    if (direction == 1)
    {
        digitalWrite(pin1, HIGH);
        digitalWrite(pin2, LOW);
    }
    else
    {
        digitalWrite(pin1, LOW);
        digitalWrite(pin2, HIGH);
    }
    analogWrite(enpin, speed);
}

void setup()
{
    // Set motor control pins as outputs
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(enablePin, OUTPUT);

    // initializing pwm signal parameters
    analogWriteResolution(enablePin, resolution);
    analogWriteFrequency(enablePin, freq);

    // ledcAttachChannel(enablePin, freq, resolution, pwmChannelL);

    delay(2000);
}

void loop()
{

    // Ramp up motor speed
    for (int speed = 0; speed <= 255; speed++)
    {
        SetMotorSpeed(1, speed, motorPin1, motorPin2, enablePin);
        delay(15); // Small delay to observe the speed change
    }
    delay(1000);

    SetMotorSpeed(1, 0, motorPin1, motorPin2, enablePin);
    // ledcWrite(pwmChannelL, 0);
    delay(2000); // Wait for 2 seconds

    // Ramp up motor speed
    for (int speed = 0; speed <= 255; speed++)
    {
        SetMotorSpeed(-1, speed, motorPin1, motorPin2, enablePin);
        delay(15); // Small delay to observe the speed change
    }
    delay(1000);

    SetMotorSpeed(1, 0, motorPin1, motorPin2, enablePin);
    // ledcWrite(pwmChannelL, 0);
    delay(2000); // Wait for 2 seconds
}