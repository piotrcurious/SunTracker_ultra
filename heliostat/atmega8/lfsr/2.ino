#include <avr/pgmspace.h>

// Configuration Constants
const float xOffset = 1.0;  // Heliostat base X-offset
const float yOffset = 2.0;  // Heliostat base Y-offset
const float zOffset = 0.5;  // Heliostat base Z-offset
const float targetX = 0.0;  // Target X-coordinate
const float targetY = 0.0;  // Target Y-coordinate
const float targetZ = 1.0;  // Target Z-coordinate
const int stepsPerDegree = 10;  // Steps per degree for motors
const float maxSpeed = 200.0;   // Maximum motor speed (steps per second)
const float acceleration = 100.0;  // Acceleration (steps per second^2)

// Hardware Pin Definitions
const int baseStepPin = 4;
const int baseDirPin = 5;
const int tiltStepPin = 6;
const int tiltDirPin = 7;
const int baseEndStopPin = 2;
const int tiltEndStopPin = 3;

// Optimized Trigonometric Lookup Tables (5-degree resolution)
const uint8_t TABLE_SIZE = 72;  // 5-degree resolution (360/5 = 72 entries)

// Precomputed sine values scaled to a range of [0, 1000] and stored in PROGMEM
const int16_t sinTable[TABLE_SIZE] PROGMEM = {
    0, 87, 173, 258, 342, 422, 500, 573, 642, 707, 766, 819, 866, 906, 939, 965,
    985, 997, 1000, 997, 985, 965, 939, 906, 866, 819, 766, 707, 642, 573, 500, 422,
    342, 258, 173, 87, 0, -87, -173, -258, -342, -422, -500, -573, -642, -707,
    -766, -819, -866, -906, -939, -965, -985, -997, -1000, -997, -985, -965,
    -939, -906, -866, -819, -766, -707, -642, -573, -500, -422, -342, -258,
    -173, -87};

// Optimized cosine table derived from the sine table
inline int16_t fastCos(int angle) {
    return fastSin(angle + 90);
}

// Lookup table helpers with second-order interpolation
float fastSin(int angle) {
    angle = angle % 360;
    if (angle < 0) angle += 360;

    int index = angle / 5;
    int nextIndex = (index + 1) % TABLE_SIZE;
    int prevIndex = (index - 1 + TABLE_SIZE) % TABLE_SIZE;

    int16_t y0 = pgm_read_word(&sinTable[prevIndex]);
    int16_t y1 = pgm_read_word(&sinTable[index]);
    int16_t y2 = pgm_read_word(&sinTable[nextIndex]);

    float t = (angle % 5) / 5.0;  // Fractional part of the angle

    // Quadratic interpolation
    return (y0 * (t - 1) * (t - 2) / 2.0 +
            y1 * (t + 1) * (t - 1) +
            y2 * (t + 1) * t / 2.0) /
           1000.0;
}

// Vector Operations
struct Vector3 {
    float x, y, z;
};

Vector3 vectorAdd(Vector3 a, Vector3 b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vector3 vectorSubtract(Vector3 a, Vector3 b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vector3 vectorScale(Vector3 v, float scalar) {
    return {v.x * scalar, v.y * scalar, v.z * scalar};
}

float vectorDot(Vector3 a, Vector3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

float vectorMagnitude(Vector3 v) {
    return sqrt(vectorDot(v, v));
}

Vector3 vectorNormalize(Vector3 v) {
    float mag = vectorMagnitude(v);
    if (mag > 0) {
        return vectorScale(v, 1.0 / mag);
    }
    return {0.0, 0.0, 0.0};
}

// Stepper Motor Control (with acceleration)
void moveStepper(int stepPin, int dirPin, int steps, float maxSpeed, float accel) {
    int direction = (steps > 0) ? HIGH : LOW;
    steps = abs(steps);
    digitalWrite(dirPin, direction);

    float speed = 0.0;
    float stepDelay;

    for (int i = 0; i < steps; i++) {
        // Acceleration phase
        if (i < steps / 2 && speed < maxSpeed) {
            speed += accel / steps;
        }
        // Deceleration phase
        if (i >= steps / 2 && speed > accel) {
            speed -= accel / steps;
        }

        stepDelay = 1000000.0 / speed;

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

// Main Program
void setup() {
    heliostatInitialize();
    Serial.begin(9600);
}

void loop() {
    float sunAzimuth = 45.0;   // Example sun azimuth in degrees
    float sunElevation = 30.0; // Example sun elevation in degrees

    heliostatUpdate(sunAzimuth, sunElevation);
    delay(1000);  // Update every second
}
