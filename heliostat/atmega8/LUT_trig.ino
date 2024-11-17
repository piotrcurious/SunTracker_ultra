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

// Trigonometric Lookup Tables
const uint8_t TABLE_SIZE = 360;  // 1-degree resolution

// Sine table
const int16_t sinTable[TABLE_SIZE] PROGMEM = {
  0, 174, 348, 523, 697, 871, 1045, 1218, 1391, 1564, 1736, 1908, 2079, 2250, 2419, 2588,
  // More precomputed values up to 359 degrees...
  -174
};

// Cosine table (same as sine table but shifted by 90 degrees)
const int16_t cosTable[TABLE_SIZE] PROGMEM = {
  1000, 999, 998, 996, 993, 990, 985, 980, 974, 967, 959, 951, 941, 930, 919, 906,
  // More precomputed values up to 359 degrees...
  -987
};

// Lookup table helpers
float fastSin(int angle) {
  angle = angle % 360;
  if (angle < 0) angle += 360;
  return pgm_read_word(&sinTable[angle]) / 1000.0;
}

float fastCos(int angle) {
  angle = angle % 360;
  if (angle < 0) angle += 360;
  return pgm_read_word(&cosTable[angle]) / 1000.0;
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

// Calculate sun direction
Vector3 calculateSunDirection(float azimuth, float elevation) {
  return {
    fastCos(azimuth) * fastCos(elevation),
    fastSin(azimuth) * fastCos(elevation),
    fastSin(elevation)
  };
}

// Calculate mirror normal
Vector3 calculateMirrorNormal(Vector3 sunDir, Vector3 targetDir) {
  return vectorNormalize(vectorAdd(sunDir, targetDir));
}

// Stepper Motor Control
void moveStepper(int stepPin, int dirPin, int steps, float speed, float accel) {
  int direction = (steps > 0) ? HIGH : LOW;
  steps = abs(steps);
  digitalWrite(dirPin, direction);

  float currentSpeed = 0.0;
  for (int i = 0; i < steps; i++) {
    if (i < steps / 2 && currentSpeed < speed) {
      currentSpeed += accel / speed;
    } else if (i >= steps / 2 && currentSpeed > accel) {
      currentSpeed -= accel / speed;
    }

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(1000000 / currentSpeed);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(1000000 / currentSpeed);
  }
}

void resetStepper(int stepPin, int dirPin, int endStopPin) {
  digitalWrite(dirPin, LOW);
  while (digitalRead(endStopPin) == HIGH) {
    digitalWrite(stepPin, HIGH);
    delay(5);
    digitalWrite(stepPin, LOW);
    delay(5);
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

  resetStepper(baseStepPin, baseDirPin, baseEndStopPin);
  resetStepper(tiltStepPin, tiltDirPin, tiltEndStopPin);
}

// Heliostat Update
void heliostatUpdate(float sunAzimuth, float sunElevation) {
  Vector3 heliostatPos = {xOffset, yOffset, zOffset};
  Vector3 targetPos = {targetX, targetY, targetZ};
  Vector3 targetDir = vectorNormalize(vectorSubtract(targetPos, heliostatPos));

  Vector3 sunDir = calculateSunDirection(sunAzimuth, sunElevation);
  Vector3 mirrorNormal = calculateMirrorNormal(sunDir, targetDir);

  float baseAngle = acos(vectorDot(mirrorNormal, {1.0, 0.0, 0.0}));
  float tiltAngle = acos(vectorDot(mirrorNormal, {0.0, 0.0, 1.0}));

  int baseSteps = round(baseAngle * stepsPerDegree);
  int tiltSteps = round(tiltAngle * stepsPerDegree);

  moveStepper(baseStepPin, baseDirPin, baseSteps, maxSpeed, acceleration);
  moveStepper(tiltStepPin, tiltDirPin, tiltSteps, maxSpeed, acceleration);
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
