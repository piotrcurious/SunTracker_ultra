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

// LFSR Configuration for Sine/Cosine
const uint16_t LFSR_MASK = 0xB400; // Polynomial: x^16 + x^14 + x^13 + x^11 + 1
uint16_t lfsrStateSin = 0xACE1;    // Initial seed for sine
uint16_t lfsrStateCos = 0x1234;    // Initial seed for cosine

// Precomputed LUT for Normalized Reflection Directions (Stored in PROGMEM)
const int16_t reflectionLUT[360][3] PROGMEM = {
    {1000, 0, 0}, // Example: normalized vector for azimuth=0, elevation=0
    // Populate with precomputed values...
};

// LFSR Function for Sine and Cosine
uint16_t lfsrNext(uint16_t &state) {
    uint16_t bit = ((state >> 0) ^ (state >> 2) ^ (state >> 3) ^ (state >> 5)) & 1;
    state = (state >> 1) | (bit << 15);
    return state;
}

int16_t lfsrSine(uint16_t angle) {
    uint16_t scaledAngle = angle % 360; // Periodicity
    uint16_t lfsrValue = lfsrNext(lfsrStateSin);
    return (lfsrValue % 2000) - 1000; // Scale to [-1000, 1000]
}

int16_t lfsrCosine(uint16_t angle) {
    uint16_t scaledAngle = angle % 360; // Periodicity
    uint16_t lfsrValue = lfsrNext(lfsrStateCos);
    return (lfsrValue % 2000) - 1000; // Scale to [-1000, 1000]
}

// Stepper Motor Control with S-Curve Acceleration
void moveStepper(int stepPin, int dirPin, int steps, int maxSpeed, int accel) {
    int direction = (steps > 0) ? HIGH : LOW;
    steps = abs(steps);
    digitalWrite(dirPin, direction);

    int stepDelay = 1000000 / maxSpeed; // Initial step delay
    for (int i = 0; i < steps; i++) {
        // Apply S-curve profile for acceleration/deceleration
        if (i < accel) {
            stepDelay = max(1000000 / (maxSpeed - accel + i), stepDelay - 1);
        } else if (i > (steps - accel)) {
            stepDelay = min(1000000 / (maxSpeed - accel + steps - i), stepDelay + 1);
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
    // Retrieve precomputed reflection vector
    int16_t reflection[3];
    memcpy_P(reflection, &reflectionLUT[sunAzimuth % 360], sizeof(reflection));

    // Adjust reflection based on sun elevation
    reflection[2] = (reflection[2] * lfsrSine(sunElevation)) / 1000;

    // Convert reflection direction to angles
    int16_t baseAngle = atan2(reflection[1], reflection[0]) * (180 / 3.14159);
    int16_t tiltAngle = atan2(reflection[2], sqrt(reflection[0] * reflection[0] + reflection[1] * reflection[1])) * (180 / 3.14159);

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
