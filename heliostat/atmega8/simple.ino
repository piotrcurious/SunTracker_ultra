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

// Helper Functions for Vector Operations
struct Vector3 {
  float x, y, z;
};

// Vector addition
Vector3 vectorAdd(Vector3 a, Vector3 b) {
  return {a.x + b.x, a.y + b.y, a.z + b.z};
}

// Vector subtraction
Vector3 vectorSubtract(Vector3 a, Vector3 b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

// Vector scaling
Vector3 vectorScale(Vector3 v, float scalar) {
  return {v.x * scalar, v.y * scalar, v.z * scalar};
}

// Vector dot product
float vectorDot(Vector3 a, Vector3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

// Vector magnitude squared
float vectorMagnitudeSquared(Vector3 v) {
  return vectorDot(v, v);
}

// Vector normalization
Vector3 vectorNormalize(Vector3 v) {
  float magSquared = vectorMagnitudeSquared(v);
  if (magSquared > 0) {
    float invMag = 1.0 / sqrt(magSquared);
    return vectorScale(v, invMag);
  }
  return {0.0, 0.0, 0.0};
}

// Angle between two vectors (cosine approximation)
float angleBetween(Vector3 v1, Vector3 v2) {
  float dot = vectorDot(v1, v2);
  float mag1 = sqrt(vectorMagnitudeSquared(v1));
  float mag2 = sqrt(vectorMagnitudeSquared(v2));
  return (mag1 > 0 && mag2 > 0) ? dot / (mag1 * mag2) : 0;
}

// Mirror normal calculation
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
    // Accelerate or decelerate
    if (i < steps / 2 && currentSpeed < speed) {
      currentSpeed += accel / speed;
    } else if (i >= steps / 2 && currentSpeed > accel) {
      currentSpeed -= accel / speed;
    }

    // Execute step
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

  // Calculate sun direction
  Vector3 sunDir = vectorNormalize({
      cos(radians(sunAzimuth)),
      sin(radians(sunAzimuth)),
      sin(radians(sunElevation))});

  // Calculate mirror normal
  Vector3 mirrorNormal = calculateMirrorNormal(sunDir, targetDir);

  // Calculate angles for stepper motors
  float baseAngle = angleBetween(mirrorNormal, {1.0, 0.0, 0.0});
  float tiltAngle = angleBetween(mirrorNormal, {0.0, 0.0, 1.0});

  // Convert angles to steps
  int baseSteps = round(baseAngle * stepsPerDegree);
  int tiltSteps = round(tiltAngle * stepsPerDegree);

  // Move steppers
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
