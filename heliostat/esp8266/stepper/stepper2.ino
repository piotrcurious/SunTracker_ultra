#include <AccelStepper.h>
#include <math.h>

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

// Galois Field and Ring Definitions
template<int MOD>
struct Field {
  int value;

  Field(int v = 0) : value((v % MOD + MOD) % MOD) {}

  Field operator+(const Field &other) const { return Field(value + other.value); }
  Field operator-(const Field &other) const { return Field(value - other.value); }
  Field operator*(const Field &other) const { return Field(value * other.value); }

  // Modular multiplicative inverse
  Field inv() const {
    int t = 0, newt = 1;
    int r = MOD, newr = value;
    while (newr != 0) {
      int quotient = r / newr;
      t = t - quotient * newt;
      std::swap(t, newt);
      r = r - quotient * newr;
      std::swap(r, newr);
    }
    if (r > 1) return Field(0); // Not invertible
    if (t < 0) t += MOD;
    return Field(t);
  }

  Field operator/(const Field &other) const { return *this * other.inv(); }
};

// Vector in homogeneous coordinates
struct Vector {
  Field<360> x, y, z, w; // Using modulo 360 for angles and homogeneous scaling

  Vector(Field<360> x, Field<360> y, Field<360> z, Field<360> w = 1)
      : x(x), y(y), z(z), w(w) {}

  Vector operator+(const Vector &v) const {
    return {x + v.x, y + v.y, z + v.z, w};
  }

  Vector operator-(const Vector &v) const {
    return {x - v.x, y - v.y, z - v.z, w};
  }

  Vector operator*(const Field<360> &scalar) const {
    return {x * scalar, y * scalar, z * scalar, w};
  }

  Vector normalize() const {
    Field<360> mag = (x * x + y * y + z * z).inv(); // 1 / ||v||
    return *this * mag;
  }
};

// Stepper motor objects
AccelStepper baseStepper(AccelStepper::DRIVER, BASE_STEP_PIN, BASE_DIR_PIN);
AccelStepper tiltStepper(AccelStepper::DRIVER, TILT_STEP_PIN, TILT_DIR_PIN);

// Function prototypes
void resetToZero();
void updateHeliostat(const Vector &sunDirection);
Field<360> calculateBaseAngle(const Vector &normal);
Field<360> calculateTiltAngle(const Vector &normal);

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
  // Global sun position (in degrees, updated externally)
  Field<360> sunAzimuth(45);   // Example value: 45 degrees
  Field<360> sunElevation(30); // Example value: 30 degrees

  // Convert sun position to a direction vector
  Vector sunDirection(
      cos(radians(sunAzimuth.value)),
      sin(radians(sunAzimuth.value)),
      sin(radians(sunElevation.value))
  );

  // Update heliostat mirror position
  updateHeliostat(sunDirection);

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
  Vector heliostatPos(
      Field<360>(config.xOffset), Field<360>(config.yOffset), Field<360>(config.zOffset)
  );
  Vector targetPos(
      Field<360>(config.targetX), Field<360>(config.targetY), Field<360>(config.targetZ)
  );

  // Calculate target direction relative to the heliostat
  Vector targetDir = (targetPos - heliostatPos).normalize();

  // Calculate mirror normal (average of sun direction and target direction)
  Vector mirrorNormal = (sunDirection + targetDir).normalize();

  // Calculate angles for steppers
  Field<360> baseAngle = calculateBaseAngle(mirrorNormal);
  Field<360> tiltAngle = calculateTiltAngle(mirrorNormal);

  // Convert angles to steps
  int baseSteps = baseAngle.value * config.stepsPerDegree;
  int tiltSteps = tiltAngle.value * config.stepsPerDegree;

  // Move steppers to calculated positions
  baseStepper.moveTo(baseSteps);
  tiltStepper.moveTo(tiltSteps);

  // Run steppers to position
  while (baseStepper.distanceToGo() != 0 || tiltStepper.distanceToGo() != 0) {
    baseStepper.run();
    tiltStepper.run();
  }
}

Field<360> calculateBaseAngle(const Vector &normal) {
  // Project mirror normal onto the X-Y plane and calculate the angle
  return Field<360>(atan2(normal.y.value, normal.x.value) * 180.0 / PI);
}

Field<360> calculateTiltAngle(const Vector &normal) {
  // Calculate elevation of the mirror normal
  return Field<360>(asin(normal.z.value) * 180.0 / PI);
}
