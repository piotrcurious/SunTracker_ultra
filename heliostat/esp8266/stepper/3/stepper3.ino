#include <AccelStepper.h>
#include <math.h>

// ----- Configuration Constants -----
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

// ----- End Stop Pins -----
#define BASE_END_STOP_PIN 2
#define TILT_END_STOP_PIN 3

// ----- Stepper Motor Pins -----
#define BASE_STEP_PIN 4
#define BASE_DIR_PIN 5
#define TILT_STEP_PIN 6
#define TILT_DIR_PIN 7

// ----- Mathematical Structures -----
template<int MOD>
struct Field {
  int value;

  Field(int v = 0) : value((v % MOD + MOD) % MOD) {}

  Field operator+(const Field &other) const { return Field(value + other.value); }
  Field operator-(const Field &other) const { return Field(value - other.value); }
  Field operator*(const Field &other) const { return Field(value * other.value); }

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
    if (r > 1) return Field(0);
    if (t < 0) t += MOD;
    return Field(t);
  }

  Field operator/(const Field &other) const { return *this * other.inv(); }
};

struct Vector {
  Field<360> x, y, z;

  Vector(Field<360> x, Field<360> y, Field<360> z) : x(x), y(y), z(z) {}

  Vector operator+(const Vector &v) const { return {x + v.x, y + v.y, z + v.z}; }
  Vector operator-(const Vector &v) const { return {x - v.x, y - v.y, z - v.z}; }
  Vector operator*(const Field<360> &scalar) const { return {x * scalar, y * scalar, z * scalar}; }

  Vector normalize() const {
    Field<360> magnitude = (x * x + y * y + z * z).inv();
    return *this * magnitude;
  }
};

// ----- Hardware Classes -----
class StepperController {
  AccelStepper stepper;

public:
  StepperController(int stepPin, int dirPin)
      : stepper(AccelStepper::DRIVER, stepPin, dirPin) {
    stepper.setMaxSpeed(config.maxSpeed);
    stepper.setAcceleration(config.acceleration);
  }

  void moveToPosition(int steps) { stepper.moveTo(steps); }

  void resetToZero(int endStopPin) {
    stepper.setSpeed(-config.maxSpeed / 2);
    while (digitalRead(endStopPin)) stepper.runSpeed();
    stepper.setCurrentPosition(0);
  }

  void runToPosition() { while (stepper.distanceToGo() != 0) stepper.run(); }
};

// ----- Heliostat Logic -----
class Heliostat {
  StepperController baseStepper;
  StepperController tiltStepper;

public:
  Heliostat()
      : baseStepper(BASE_STEP_PIN, BASE_DIR_PIN), tiltStepper(TILT_STEP_PIN, TILT_DIR_PIN) {}

  void initialize() {
    pinMode(BASE_END_STOP_PIN, INPUT_PULLUP);
    pinMode(TILT_END_STOP_PIN, INPUT_PULLUP);
    baseStepper.resetToZero(BASE_END_STOP_PIN);
    tiltStepper.resetToZero(TILT_END_STOP_PIN);
  }

  void updatePosition(const Vector &sunDirection) {
    Vector heliostatPos = {config.xOffset, config.yOffset, config.zOffset};
    Vector targetPos = {config.targetX, config.targetY, config.targetZ};
    Vector targetDir = (targetPos - heliostatPos).normalize();
    Vector mirrorNormal = (sunDirection + targetDir).normalize();

    int baseSteps = calculateBaseAngle(mirrorNormal).value * config.stepsPerDegree;
    int tiltSteps = calculateTiltAngle(mirrorNormal).value * config.stepsPerDegree;

    baseStepper.moveToPosition(baseSteps);
    tiltStepper.moveToPosition(tiltSteps);
    baseStepper.runToPosition();
    tiltStepper.runToPosition();
  }

private:
  Field<360> calculateBaseAngle(const Vector &normal) {
    return Field<360>(atan2(normal.y.value, normal.x.value) * 180.0 / PI);
  }

  Field<360> calculateTiltAngle(const Vector &normal) {
    return Field<360>(asin(normal.z.value) * 180.0 / PI);
  }
};

// ----- Main Program -----
Heliostat heliostat;

void setup() {
  Serial.begin(115200);
  heliostat.initialize();
  Serial.println("Heliostat initialized.");
}

void loop() {
  Field<360> sunAzimuth(45);   // Example: 45 degrees
  Field<360> sunElevation(30); // Example: 30 degrees
  Vector sunDirection(
      cos(radians(sunAzimuth.value)),
      sin(radians(sunAzimuth.value)),
      sin(radians(sunElevation.value))
  );

  heliostat.updatePosition(sunDirection);

  delay(1000); // Update interval
}
