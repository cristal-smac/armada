// NOT WORKING for now !!!!!!!!
//  Motor control pins
const int motorPin1 = 2;    // IN3 on the motor driver
const int motorPin2 = 4;    // IN4 on the motor driver
const int enablePin = 18;   // ENB on the motor driver
const int encoderPinA = 34; // Encoder output A
const int encoderPinB = 35; // Encoder output B

// pwm parameters setup
const int freq = 30000;
const int pwmChannelL = 1;
const int resolution = 8;

// PID constants
float kp = 1.8;
float ki = 5;
float kd = 0.1;

// PID variables
volatile long encoderCount = 0;
float previousError = 0;
float integral = 0;
unsigned long previousTime = 0;
long CurrentTimeforError;
long PreviousTimeForError;

// Motor parameters
const int ticksPerRevolution = 48;

void SetMotorSpeed(int direction, int speed, int pin1, int pin2, int enpin)
{ // set motor speed in pwm
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
    Serial.begin(115200);

    // motor control pins
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(enablePin, OUTPUT);

    // encoder pins
    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);

    // interrupt for encoder pin A
    // attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, RISING);

    // initializing pwm signal parameters
    analogWriteResolution(enablePin, resolution);
    analogWriteFrequency(enablePin, freq);
}

void loop()
{
    // Desired motor speed in RPM
    float setpoint = 50.0;
    bool targetReached = false;

    // Ramp up to the desired speed
    while (!targetReached)
    {
        float currentRPM = getRPM();
        Serial.print("RPM: ");
        Serial.println(currentRPM);
        float controlSignal = pidControl(setpoint, currentRPM);
        // setMotorSpeed(controlSignal);
        SetMotorSpeed(1, controlSignal, motorPin1, motorPin2, enablePin);

        // Check if the motor has reached the target speed
        if (abs(currentRPM - setpoint) < 5.0)
        { // Allow a small tolerance
            targetReached = true;
        }

        delay(100); // Small delay for stability
    }
    delay(2000);
    SetMotorSpeed(1, 0, motorPin1, motorPin2, enablePin); // then we stop the motor
}

// Interrupt service routine to update encoder count
void updateEncoder()
{
    if (digitalRead(encoderPinB) > digitalRead(encoderPinA))
    {
        encoderCount++;
    }
    else
    {
        encoderCount--;
    }

    // Serial.print("Encoder Count: ");
    // Serial.println(encoderCount);
}

// Calculate motor RPM
float getRPM()
{

    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0;
    float revolutions = static_cast<float>(encoderCount) / ticksPerRevolution;
    float rpm = revolutions * (60.0 / deltaTime);
    encoderCount = 0;
    previousTime = currentTime;
    return rpm;
}

// PID control function
float pidControl(float setpoint, float measuredValue)
{
    CurrentTimeforError = millis();
    float delta2 = ((float)CurrentTimeforError - PreviousTimeForError) / 1.0e3;
    float error = setpoint - measuredValue;
    integral = integral + (error * delta2);
    float derivative = (error - previousError) / delta2;
    float controlSignal = (kp * error) + (ki * integral) + (kd * derivative);

    previousError = error;
    PreviousTimeForError = CurrentTimeforError;
    return controlSignal;
}
