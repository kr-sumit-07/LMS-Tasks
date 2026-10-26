#include <Arduino.h>

// --- Pin Definitions ---
const uint8_t SENSOR_PINS[5] = {32, 33, 34, 35, 36}; // ESP32 ADC Pins (S1 to S5)

const uint8_t MOTOR_LEFT_PWM  = 25;
const uint8_t MOTOR_LEFT_DIR  = 26;
const uint8_t MOTOR_RIGHT_PWM = 27;
const uint8_t MOTOR_RIGHT_DIR = 14;

// --- Controller Parameters ---
float Kp = 120.0f;
float Ki = 0.0f;      // Kept at 0 for PD control
float Kd = 25.0f;

const int BASE_SPEED = 180;   // Base PWM speed (0 - 255)
const int MAX_SPEED  = 255;   // Max PWM limit

// --- Sensor Weights (-2 to +2) ---
const float SENSOR_WEIGHTS[5] = {-2.0f, -1.0f, 0.0f, 1.0f, 2.0f};

// --- State Variables ---
float lastError = 0.0f;
float lastValidError = 0.0f;
unsigned long lastTime = 0;
bool lineLost = false;

// --- Function Declarations ---
bool readSensorsAndGetError(float &error);
void applyMotorCommands(int leftSpeed, int rightSpeed);
void handleLostLine();

void setup() {
    Serial.begin(115200);

    // Initialize Motor Pins
    pinMode(MOTOR_LEFT_PWM, OUTPUT);
    pinMode(MOTOR_LEFT_DIR, OUTPUT);
    pinMode(MOTOR_RIGHT_PWM, OUTPUT);
    pinMode(MOTOR_RIGHT_DIR, OUTPUT);

    // Initialize Sensor Pins
    for (int i = 0; i < 5; i++) {
        pinMode(SENSOR_PINS[i], INPUT);
    }

    lastTime = micros();
    Serial.println("PID Line Follower Initialized.");
}

void loop() {
    unsigned long currentTime = micros();
    float dt = (currentTime - lastTime) / 1000000.0f; // Convert to seconds
    if (dt <= 0.0f) dt = 0.001f; // Prevent division by zero

    float error = 0.0f;
    bool hasLine = readSensorsAndGetError(error);

    if (hasLine) {
        lineLost = false;
        lastValidError = error;

        // --- PD Controller Calculation ---
        float pTerm = Kp * error;
        float dTerm = Kd * ((error - lastError) / dt);
        float correction = pTerm + dTerm;

        // Calculate Motor Speeds
        int leftMotorSpeed  = BASE_SPEED + (int)correction;
        int rightMotorSpeed = BASE_SPEED - (int)correction;

        // Clamp Speed Values
        leftMotorSpeed  = constrain(leftMotorSpeed, -MAX_SPEED, MAX_SPEED);
        rightMotorSpeed = constrain(rightMotorSpeed, -MAX_SPEED, MAX_SPEED);

        applyMotorCommands(leftMotorSpeed, rightMotorSpeed);

        lastError = error;
        lastTime = currentTime;
    } else {
        // --- Lost Line Recovery Routine ---
        handleLostLine();
    }

    delay(2); // Control loop execution delay (~500Hz)
}

/**
 * Calculates weighted centroid position error from 5 ADC sensors.
 * Returns true if line detected, false if line lost (all zero / off track).
 */
bool readSensorsAndGetError(float &error) {
    float weightedSum = 0.0f;
    float totalSum = 0.0f;

    for (int i = 0; i < 5; i++) {
        int rawReading = analogRead(SENSOR_PINS[i]);
        
        // Edge check: Clamp ADC bounds (0 - 4095)
        if (rawReading < 0) rawReading = 0;
        if (rawReading > 4095) rawReading = 4095;

        weightedSum += (float)rawReading * SENSOR_WEIGHTS[i];
        totalSum += (float)rawReading;
    }

    // Zero-division / Lost-Line Check
    if (totalSum < 100.0f) { // Threshold for all sensors reading background
        return false;
    }

    // Position range: -2.0 (Far Left) to +2.0 (Far Right)
    error = weightedSum / totalSum;
    return true;
}

/**
 * Applies speed and direction outputs to motor drivers.
 */
void applyMotorCommands(int leftSpeed, int rightSpeed) {
    // Direction set for forward motion
    digitalWrite(MOTOR_LEFT_DIR, HIGH);
    digitalWrite(MOTOR_RIGHT_DIR, HIGH);

    analogWrite(MOTOR_LEFT_PWM, leftSpeed);
    analogWrite(MOTOR_RIGHT_PWM, rightSpeed);
}

/**
 * Executes in-place recovery spin based on the last known line position.
 */
void handleLostLine() {
    lineLost = true;
    
    // Memory-based spin towards the last known line location
    if (lastValidError < 0.0f) {
        // Line was on the left -> Spin Left in-place
        digitalWrite(MOTOR_LEFT_DIR, LOW);   // Reverse Left Motor
        digitalWrite(MOTOR_RIGHT_DIR, HIGH); // Forward Right Motor
        analogWrite(MOTOR_LEFT_PWM, 130);
        analogWrite(MOTOR_RIGHT_PWM, 130);
    } else {
        // Line was on the right -> Spin Right in-place
        digitalWrite(MOTOR_LEFT_DIR, HIGH);  // Forward Left Motor
        digitalWrite(MOTOR_RIGHT_DIR, LOW);  // Reverse Right Motor
        analogWrite(MOTOR_LEFT_PWM, 130);
        analogWrite(MOTOR_RIGHT_PWM, 130);
    }
}