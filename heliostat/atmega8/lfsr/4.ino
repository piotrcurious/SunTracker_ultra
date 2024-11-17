#include <avr/io.h>
#include <avr/pgmspace.h>

// Configuration Constants
const int16_t xOffset = 1000;  // Fixed-point scale: X-offset (1.0 = 1000)
const int16_t yOffset = 2000;  // Fixed-point scale: Y-offset (2.0 = 2000)
const int16_t zOffset = 500;   // Fixed-point scale: Z-offset (0.5 = 500)
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

// LFSR Configuration
const uint16_t LFSR_MASK = 0xB400; // Polynomial: x^16 + x^14 + x^13 + x^11 + 1
uint16_t lfsrState = 0xACE1;      // Initial seed

// Function to generate the next value in the LFSR sequence
uint16_t lfsrNext() {
    uint16_t bit = ((lfsrState >> 0) ^ (lfsrState >> 2) ^ (lfsrState >> 3) ^ (lfsrState >> 5)) & 1;
    lfsrState = (lfsrState >> 1) | (bit << 15);
    return lfsrState;
}

// Generate scaled sine value using LFSR
int16_t lfsrSine(uint16_t angle) {
    uint16_t sequence = angle % 360; // Ensure periodicity
    uint16_t lfsrValue = lfsrNext();
    return (lfsrValue % 2000) - 1000; // Scale to [-1000, 1000]
}

// Generate scaled cosine value using LFSR (90-degree phase shift)
int16_t lfsrCosine(uint16_t angle) {
    return lfsrSine(angle + 90);
}

// Vector Operations (Fixed-Point Arithmetic)
struct Vector3 {
    int16_t x, y, z; // Fixed-point representation
};

Vector3 matrixVectorMultiply(const int16_t matrix[3][3], Vector3 v) {
    return {
        (matrix[0][0] * v.x + matrix[0][1] * v.y + matrix[0][2] * v.z) / 1000,
        (matrix[1][0] * v.x + matrix[1][1] * v.y + matrix[1][2] * v.z) / 1000,
        (matrix[2][0] * v.x + matrix[2][1] * v.y + matrix[2][2] * v.z) / 1000
    };
}

// Stepper Motor Control (with acceleration using Bresenham's algorithm)
void moveStepper(int stepPin, int dirPin, int steps, int maxSpeed, int accel) {
    int direction = (steps > 0) ? HIGH : LOW;
    steps = abs(steps);
    digitalWrite(dirPin, direction);

    int stepDelay = 1000000 / maxSpeed; // Initial step delay
    for (int i = 0; i < steps; i++) {
        if (i < accel && stepDelay > 1000000 / accel) { // Ramp-up
            stepDelay -= 100;
        } else if (i > (steps - accel) && stepDelay < 1000000 / accel) { // Ramp-down
            stepDelay += 100;
        }

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
    Vector3 sunDir = {
        lfsrCosine(sunAzimuth),
        lfsrSine(sunAzimuth),
        lfsrSine(sunElevation)
    };
    
    Vector3 target = {0 - xOffset, 0 - yOffset, 1000 - zOffset};
    
    // Normalize target vector
    int16_t magnitude = sqrt(target.x * target.x + target.y * target.y + target.z * target.z);
    target.x = (target.x * 1000) / magnitude;
    target.y = (target.y * 1000) / magnitude;
    target.z = (target.z * 1000) / magnitude;
    
    // Reflection direction = 2*(dot(normal, sunDir))*normal - sunDir
    int16_t dotProduct = (target.x * sunDir.x + target.y * sunDir.y + target.z * sunDir.z) / 1000;
    Vector3 reflectionDir = {
        2 * dotProduct * target.x - sunDir.x,
        2 * dotProduct * target.y - sunDir.y,
        2 * dotProduct * target.z - sunDir.z
    };

    // Convert reflection direction to angles
    int16_t baseAngle = atan2(reflectionDir.y, reflectionDir.x) * (180 / 3.14159);
    int16_t tiltAngle = atan2(reflectionDir.z, sqrt(reflectionDir.x * reflectionDir.x + reflectionDir.y * reflectionDir.y)) * (180 / 3.14159);

    // Move motors
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
