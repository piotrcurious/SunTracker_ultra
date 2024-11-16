#include <AccelStepper.h>

// Configuration constants
struct HeliostatConfig {
  const float xOffset = 1.0; // X Offset in meters
  const float yOffset = 2.0; // Y Offset in meters
  const float zOffset = 0.5; // Z Offset in meters
  const float targetX = 0.0; // Target X position in meters
  const float targetY = 0.0; // Target Y position in meters
  const float targetZ = 1.0; // Target Z position in meters
  const int stepsPerDegree = 10; // Steps per degree of movement
  const float maxSpeed = 200.0; // Max speed in steps/second
  const float acceleration = 100.0; // Acceleration in steps/second^2
};

const HeliostatConfig config;

// End stop pins
#define BASE_END_STOP_PIN 2
#define TILT_END_STOP_PIN 3

// Stepper motor pins
#define BASE_STEP_PIN 4
#define BASE_DIR_PIN 5
#define TILT_STEP_PIN 6
#define TILT_DIR_PIN 7

// Global sun position (updated externally)
float sunAzimuth = 0.0;   // in degrees
float sunElevation = 0.0; // in degrees

// Vector structure for geometry
struct Vector {
  float x, y, z;

  Vector operator+(const Vector &v) const { return {x + v.x, y + v.y, z + v.z}; }
  Vector operator-(const Vector &v) const { return {x - v.x, y - v.y, z - v.z}; }
  Vector operator*(float scalar) const { return {x * scalar, y * scalar, z * scalar}; }
  Vector normalize() const {
    float mag = sqrt(x * x + y * y + z * z);
    return {x / mag, y / mag, z / mag};
  }
};

// Stepper motor objects
AccelStepper baseStepper(AccelStepper::DRIVER, BASE_STEP_PIN, BASE_DIR_PIN);
AccelStepper tiltStepper(AccelStepper::DRIVER, TILT_STEP_PIN, TILT_DIR_PIN);

// Function prototypes
void resetToZero();
void updateHeliostat(const Vector &sunDirection);
float calculateBaseAngle(const Vector &normal);
float calculateTiltAngle(const Vector &normal);

void setup() {
  // Configure end stop pins
  pinMode(BASE_END_STOP_PIN, INPUT_PULLUP);
  pinMode(TILT_END_STOP_PIN, INPUT_PULLUP);

  // Configure stepper motors
  baseStepper.setMaxSpeed(config.maxSpeed);
  baseStepper.setAcceleration(config.acceleration);
  tiltStepper.setMaxSpeed(config.maxSpeed);
  tiltStepper.setAcceleration(config.acceleration);

  // Reset steppers to zero
  resetToZero();

  Serial.begin(115200);
  Serial.println("Heliostat initialized.");
}

void loop() {
  // Convert sun position to a direction vector
  float sunAzimuthRad = radians(sunAzimuth);
  float sunElevationRad = radians(sunElevation);

  Vector sunDirection = {
    cos(sunElevationRad) * cos(sunAzimuthRad),
    cos(sunElevationRad) * sin(sunAzimuthRad),
    sin(sunElevationRad)
  };

  // Update heliostat mirror position
  updateHeliostat(sunDirection);

  // Debug output
  Serial.print("Sun Azimuth: ");
  Serial.print(sunAzimuth);
  Serial.print(" | Sun Elevation: ");
  Serial.println(sunElevation);

  delay(1000); // Update interval
}

void resetToZero() {
  Serial.println("Resetting to zero position...");

  // Move base stepper until end stop is triggered
  baseStepper.setSpeed(-config.maxSpeed / 2); // Reverse direction
  while (digitalRead(BASE_END_STOP_PIN)) {
    baseStepper.runSpeed();
  }
  baseStepper.setCurrentPosition(0);

  // Move tilt stepper until end stop is triggered
  tiltStepper.setSpeed(-config.maxSpeed / 2); // Reverse direction
  while (digitalRead(TILT_END_STOP_PIN)) {
    tiltStepper.runSpeed();
  }
  tiltStepper.setCurrentPosition(0);

  Serial.println("Zero position reached.");
}

void updateHeliostat(const Vector &sunDirection) {
  Vector heliostatPos = {config.xOffset, config.yOffset, config.zOffset};
  Vector targetPos = {config.targetX, config.targetY, config.targetZ};

  // Calculate target direction relative to the heliostat
  Vector targetDir = (targetPos - heliostatPos).normalize();

  // Calculate mirror normal (average of sun direction and target direction)
  Vector mirrorNormal = (sunDirection + targetDir).normalize();

  // Calculate angles for steppers
  float baseAngle = calculateBaseAngle(mirrorNormal);
  float tiltAngle = calculateTiltAngle(mirrorNormal);

  // Convert angles to steps
  int baseSteps = baseAngle * config.stepsPerDegree;
  int tiltSteps = tiltAngle * config.stepsPerDegree;

  // Move steppers to calculated positions
  baseStepper.moveTo(baseSteps);
  tiltStepper.moveTo(tiltSteps);

  // Run steppers to position
  while (baseStepper.distanceToGo() != 0 || tiltStepper.distanceToGo() != 0) {
    baseStepper.run();
    tiltStepper.run();
  }
}

float calculateBaseAngle(const Vector &normal) {
  // Project mirror normal onto the X-Y plane and calculate the angle
  float angle = atan2(normal.y, normal.x) * 180.0 / PI;
  return constrain(angle + 90, 0, 180); // Adjust for physical constraints
}

float calculateTiltAngle(const Vector &normal) {
  // Calculate elevation of the mirror normal
  float angle = asin(normal.z) * 180.0 / PI;
  return constrain(angle + 90, 0, 180); // Adjust for physical constraints
}
