#include <avr/io.h>
#include <avr/pgmspace.h>
#include <math.h>

// Configuration Constants
const int16_t xOffset = 1000;  // Fixed-point scale: X-offset (1.0 = 1000)
const int16_t yOffset = 2000;  // Fixed-point scale: Y-offset (2.0 = 2000)
const int16_t zOffset = 500;   // Fixed-point scale: Z-offset (0.5 = 500)
const int16_t targetX = 0;     // Fixed-point scale: Target X-coordinate
const int16_t targetY = 0;     // Fixed-point scale: Target Y-coordinate
const int16_t targetZ = 1000;  // Fixed-point scale: Target Z-coordinate
const int stepsPerDegree = 10; // Steps per degree for motors
const int maxSpeed = 200;      // Maximum motor speed (steps per second)
const int acceleration = 100;  // Acceleration (steps per second^2)

// Hardware Pin Definitions
const int baseStepPin = 4;
const int baseDirPin = 5;
const int tiltStepPin = 6;
const int tiltDirPin = 7;
const int baseEndStopPin = 2;
const int tiltEndStopPin = 3;

// LFSR Configuration for Sine Wave Approximation
const uint16_t LFSR_MASK = 0xB400; // LFSR polynomial: x^16 + x^14 + x^13 + x^11 + 1
uint16_t lfsr_state = 0xACE1;      // Initial seed

// Function to generate the next value in the LFSR sequence
uint16_t lfsrNext() {
    uint16_t bit = ((lfsr_state >> 0) ^ (lfsr_state >> 2) ^ (lfsr_state >> 3) ^ (lfsr_state >> 5)) & 1;
    lfsr_state = (lfsr_state >> 1) | (bit << 15);
    return lfsr_state;
}

// Generate scaled sine value using LFSR
int16_t lfsrSine(uint16_t angle) {
    uint16_t sequence = angle % 360; // Ensure periodicity
    uint16_t lfsr_value = lfsrNext();
    return (lfsr_value % 2000) - 1000; // Scale to [-1000, 1000]
}

// Generate scaled cosine value using LFSR
int16_t lfsrCosine(uint16_t angle) {
    return lfsrSine(angle + 90); // Phase shift for cosine
}

// Vector Operations (Fixed-Point Arithmetic)
struct Vector3 {
    int16_t x, y, z; // Fixed-point representation
};

Vector3 vectorAdd(Vector3 a, Vector3 b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vector3 vectorSubtract(Vector3 a, Vector3 b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vector3 vectorScale(Vector3 v, int16_t scalar) {
    return {(v.x * scalar) / 1000, (v.y * scalar) / 1000, (v.z * scalar) / 1000};
}

int16_t vectorDot(Vector3 a, Vector3 b) {
    return (a.x * b.x + a.y * b.y + a.z * b.z) / 1000;
}

int16_t vectorMagnitude(Vector3 v) {
    return sqrt(vectorDot(v, v));
}

Vector3 vectorNormalize(Vector3 v) {
    int16_t mag = vectorMagnitude(v);
    if (mag > 0) {
        return vectorScale(v, 1000 / mag);
    }
    return {0, 0, 0};
}

// Stepper Motor Control (with acceleration)
void moveStepper(int stepPin, int dirPin, int steps, int maxSpeed, int accel) {
    int direction = (steps > 0) ? HIGH : LOW;
    steps = abs(steps);
    digitalWrite(dirPin, direction);

    int speed = 0;
    int stepDelay;

    for (int i = 0; i < steps; i++) {
        // Acceleration phase
        if (i < steps / 2 && speed < maxSpeed) {
            speed += accel / steps;
        }
        // Deceleration phase
        if (i >= steps / 2 && speed > accel) {
            speed -= accel / steps;
        }

        stepDelay = 1000000 / speed;

        digitalWrite(stepPin, HIGH);
        delayMicroseconds(stepDelay / 2);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(stepDelay / 2);
    }
}

// Heliostat Initialization
void heliostatInitialize() {
    pinMode(baseStepPin, OUTPUT);
    pinMode(baseDirPin, OUTPUT);
    pinMode(tiltStepPin, OUTPUT);
    pinMode(tiltDirPin, OUTPUT);
    pinMode(baseEndStopPin, INPUT_PULLUP);
    pinMode(tiltEndStopPin, INPUT_PULLUP);

    // Reset to end stops
    while (digitalRead(baseEndStopPin) == HIGH) {
        moveStepper(baseStepPin, baseDirPin, -1, 10, 5);
    }
    while (digitalRead(tiltEndStopPin) == HIGH) {
        moveStepper(tiltStepPin, tiltDirPin, -1, 10, 5);
    }
}

// Update Heliostat Position
void heliostatUpdate(int16_t sunAzimuth, int16_t sunElevation) {
    Vector3 sunDir = {lfsrCosine(sunAzimuth), lfsrSine(sunAzimuth), lfsrSine(sunElevation)};
    Vector3 mirrorNormal = vectorNormalize(vectorSubtract({targetX, targetY, targetZ}, {xOffset, yOffset, zOffset}));
    Vector3 reflectionDir = vectorSubtract(vectorScale(mirrorNormal, 2 * vectorDot(mirrorNormal, sunDir)), sunDir);

    int16_t baseAngle = atan2(reflectionDir.y, reflectionDir.x) * (180 / 3.14159);
    int16_t tiltAngle = atan2(reflectionDir.z, vectorMagnitude({reflectionDir.x, reflectionDir.y, 0})) * (180 / 3.14159);

    moveStepper(baseStepPin, baseDirPin, baseAngle * stepsPerDegree, maxSpeed, acceleration);
    moveStepper(tiltStepPin, tiltDirPin, tiltAngle * stepsPerDegree, maxSpeed, acceleration);
}

// Main Program
void setup() {
    heliostatInitialize();
    Serial.begin(9600);
}

void loop() {
    int16_t sunAzimuth = 4500;  // Example sun azimuth (45.00 degrees)
    int16_t sunElevation = 3000; // Example sun elevation (30.00 degrees)

    heliostatUpdate(sunAzimuth, sunElevation);
    delay(1000); // Update every second
}
