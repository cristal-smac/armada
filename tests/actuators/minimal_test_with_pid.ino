// Motor control pins
const int motorPin1 = 26;   // IN1 on the motor driver
const int motorPin2 = 27;   // IN2 on the motor driver
const int enablePin = 25;   // ENA on the motor driver
const int encoderPinA = 18; // Encoder output A
const int encoderPinB = 19; // Encoder output B

// PID constants
float kp = 1.8;
float ki = 5;
float kd = 0.1;

// PID variables
volatile long encoderCount = 0;
float previousError = 0;
float integral = 0;
unsigned long previousTime = 0;

// Motor parameters
const int ticksPerRevolution = 48;

void setup()
{
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

    Serial.begin(115200);
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
        setMotorSpeed(controlSignal);

        // Check if the motor has reached the target speed
        if (abs(currentRPM - setpoint) < 5.0)
        { // Allow a small tolerance
            targetReached = true;
        }

        delay(100); // Small delay for stability
    }
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
    float error = setpoint - measuredValue;
    Serial.print("ERROR:");
    Serial.println(error);
    integral += error;
    float derivative = error - previousError;
    float controlSignal = (kp * error) + (ki * integral) + (kd * derivative);
    previousError = error;
    return controlSignal;
}

// Set motor speed based on control signal
void setMotorSpeed(float controlSignal)
{
    int motorSpeed = constrain(abs(controlSignal), 0, 255);
    if (controlSignal > 0)
    {
        digitalWrite(motorPin1, HIGH);
        digitalWrite(motorPin2, LOW);
    }
    else
    {
        digitalWrite(motorPin1, LOW);
        digitalWrite(motorPin2, HIGH);
    }
    analogWrite(enablePin, motorSpeed);
}

// Stop the motor
void stopMotor()
{
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    analogWrite(enablePin, 0);
}