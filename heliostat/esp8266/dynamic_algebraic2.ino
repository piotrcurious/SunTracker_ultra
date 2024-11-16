#include <Servo.h>
#include <math.h>

// Constants for heliostat configuration
struct HeliostatConfig {
  const float xOffset = 1.0; // X Offset in meters
  const float yOffset = 2.0; // Y Offset in meters
  const float zOffset = 0.5; // Z Offset in meters
  const float targetX = 0.0; // Target X position in meters
  const float targetY = 0.0; // Target Y position in meters
  const float targetZ = 1.0; // Target Z position in meters
};

const HeliostatConfig config;

// Vector structure for algebraic geometry
struct Vector {
  float x, y, z;

  // Vector addition
  Vector operator+(const Vector &v) const {
    return {x + v.x, y + v.y, z + v.z};
  }

  // Vector subtraction
  Vector operator-(const Vector &v) const {
    return {x - v.x, y - v.y, z - v.z};
  }

  // Scale vector by scalar
  Vector operator*(float scalar) const {
    return {x * scalar, y * scalar, z * scalar};
  }

  // Normalize vector
  Vector normalize() const {
    float mag = sqrt(x * x + y * y + z * z);
    return {x / mag, y / mag, z / mag};
  }

  // Dot product
  float dot(const Vector &v) const {
    return x * v.x + y * v.y + z * v.z;
  }

  // Cross product
  Vector cross(const Vector &v) const {
    return {
      y * v.z - z * v.y,
      z * v.x - x * v.z,
      x * v.y - y * v.x
    };
  }
};

// Heliostat class
class Heliostat {
public:
  Heliostat(uint8_t basePin, uint8_t tiltPin)
      : baseServoPin(basePin), tiltServoPin(tiltPin) {}

  void init() {
    baseServo.attach(baseServoPin);
    tiltServo.attach(tiltServoPin);
    setMirrorAngles(90, 90); // Default neutral position
  }

  void updateMirror(const Vector &sunDir) {
    Vector heliostatPos = {config.xOffset, config.yOffset, config.zOffset};
    Vector targetPos = {config.targetX, config.targetY, config.targetZ};

    // Calculate target direction relative to the heliostat
    Vector targetDir = (targetPos - heliostatPos).normalize();

    // Calculate mirror normal (average of sun direction and target direction)
    Vector mirrorNormal = (sunDir + targetDir).normalize();

    // Calculate base and tilt angles
    float baseAngle = calculateBaseAngle(mirrorNormal);
    float tiltAngle = calculateTiltAngle(mirrorNormal);

    // Actuate the servos
    setMirrorAngles(baseAngle, tiltAngle);
  }

private:
  Servo baseServo;
  Servo tiltServo;
  uint8_t baseServoPin;
  uint8_t tiltServoPin;

  float calculateBaseAngle(const Vector &normal) {
    // Project mirror normal onto the X-Y plane and calculate the angle
    float angle = atan2(normal.y, normal.x) * 180.0 / PI;
    return constrain(angle + 90, 0, 180); // Adjust for servo limits
  }

  float calculateTiltAngle(const Vector &normal) {
    // Calculate elevation of the mirror normal
    float angle = asin(normal.z) * 180.0 / PI;
    return constrain(angle + 90, 0, 180); // Adjust for servo limits
  }

  void setMirrorAngles(float baseAngle, float tiltAngle) {
    baseServo.write(baseAngle);
    tiltServo.write(tiltAngle);
  }
};

// Global sun position (updated externally)
float sunAzimuth = 0.0;   // in degrees
float sunElevation = 0.0; // in degrees

// Heliostat object
Heliostat heliostat(5, 6); // Servo pins

void setup() {
  Serial.begin(115200);
  heliostat.init();
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

  // Update the heliostat mirror position
  heliostat.updateMirror(sunDirection);

  // Debug output
  Serial.print("Sun Azimuth: ");
  Serial.print(sunAzimuth);
  Serial.print(" | Sun Elevation: ");
  Serial.println(sunElevation);

  delay(1000); // Update interval
}
