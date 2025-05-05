#include <Arduino.h>
#include <cmath> // Use C++ standard math library
// #include <Servo.h> // Standard Arduino Servo library
#include <ESP32Servo.h> // Recommended for ESP32 hardware PWM

// --- Constants ---
// Define PI if not provided by cmath (M_PI is common)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --- Configuration ---
// Coordinate System Assumption:
// +X: East
// +Y: North
// +Z: Up
// Azimuth: Angle from North (Y-axis), positive towards East (X-axis) -> 0 = North, 90 = East
// Elevation: Angle from the horizontal XY plane, positive upwards (towards +Z)
struct HeliostatConfig {
  const float xOffset = 1.0;     // Heliostat X position relative to origin (meters)
  const float yOffset = 2.0;     // Heliostat Y position relative to origin (meters)
  const float zOffset = 0.5;     // Heliostat Z position (height) relative to origin (meters)
  const float targetX = 0.0;     // Target X position relative to origin (meters)
  const float targetY = 0.0;     // Target Y position relative to origin (meters)
  const float targetZ = 1.0;     // Target Z position (height) relative to origin (meters)

  // Calibration: Adjust these offsets based on your physical setup!
  // Base/Azimuth: Angle offset to align servo 0/90 deg with coordinate system North/East
  const float baseAngleOffset = 90.0; // Example: If servo 0 deg points North (+Y), atan2 result (relative to +X) needs offset
  // Tilt/Elevation: Angle offset to align servo 0/90 deg with horizontal/vertical
  const float tiltAngleOffset = 90.0; // Example: If servo 0 deg is horizontal, 90 is up, asin result [-90, 90] needs mapping to [0, 180]

  // Servo range constraints
  const float minServoAngle = 0.0;
  const float maxServoAngle = 180.0;
};

const HeliostatConfig config; // Global config instance

// --- Vector Structure ---
struct Vector {
  float x, y, z;

  // Default constructor
  Vector() : x(0.0f), y(0.0f), z(0.0f) {}
  // Parameterized constructor
  Vector(float x, float y, float z) : x(x), y(y), z(z) {}


  Vector operator+(const Vector &v) const { return {x + v.x, y + v.y, z + v.z}; }
  Vector operator-(const Vector &v) const { return {x - v.x, y - v.y, z - v.z}; }
  Vector operator*(float scalar) const { return {x * scalar, y * scalar, z * scalar}; }

  float magnitudeSquared() const { return x * x + y * y + z * z; }
  float magnitude() const { return sqrt(magnitudeSquared()); }

  Vector normalize() const {
    float mag = magnitude();
    // Prevent division by zero or near-zero
    if (mag < 1e-6) { // Use a small epsilon
      return {0, 0, 0}; // Return zero vector if magnitude is negligible
    }
    return {x / mag, y / mag, z / mag};
  }

  float dot(const Vector &v) const { return x * v.x + y * v.y + z * v.z; }

  Vector cross(const Vector &v) const {
    return {y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x};
  }
};

// --- Heliostat Class ---
class Heliostat {
public:
  // Constructor takes const pin numbers
  Heliostat(const uint8_t basePin, const uint8_t tiltPin)
      : baseServoPin(basePin), tiltServoPin(tiltPin) {}

  void init() {
    // Store fixed positions calculated once
    heliostatPos = {config.xOffset, config.yOffset, config.zOffset};
    Vector targetPos = {config.targetX, config.targetY, config.targetZ};
    // Calculate and store the constant vector pointing from heliostat to target
    targetDir = (targetPos - heliostatPos).normalize();

    // Attach servos using ESP32Servo library
    // Allow allocation of all timers
	  ESP32PWM::allocateTimer(0);
	  ESP32PWM::allocateTimer(1);
	  ESP32PWM::allocateTimer(2);
	  ESP32PWM::allocateTimer(3);
    baseServo.setPeriodHertz(50); // Standard servo frequency
    tiltServo.setPeriodHertz(50);

    baseServo.attach(baseServoPin, 500, 2500); // Attach with common pulse width range
    tiltServo.attach(tiltServoPin, 500, 2500); // Attach with common pulse width range

    // Set initial position (e.g., pointing horizontally North or East)
    // This depends heavily on your calibration offsets. 90/90 is often a safe start.
    setMirrorAngles(90.0, 90.0);
    Serial.println("Heliostat initialized.");
    Serial.printf("  Heliostat Pos: (%.2f, %.2f, %.2f)\n", heliostatPos.x, heliostatPos.y, heliostatPos.z);
    Serial.printf("  Target Dir:    (%.3f, %.3f, %.3f)\n", targetDir.x, targetDir.y, targetDir.z);

  }

  // Update mirror orientation based on sun direction vector
  void updateMirror(const Vector &sunDir) {
    // Calculate the required mirror normal vector
    // Normal = halfway vector between sun direction and target direction
    Vector mirrorNormal = (sunDir + targetDir).normalize();

    // Calculate required servo angles from the mirror normal
    float baseAngle = calculateBaseAngle(mirrorNormal);
    float tiltAngle = calculateTiltAngle(mirrorNormal);

    // Debug output for calculated values
    Serial.printf("  Sun Dir:    (%.3f, %.3f, %.3f)\n", sunDir.x, sunDir.y, sunDir.z);
    Serial.printf("  Mirror Norm:(%.3f, %.3f, %.3f)\n", mirrorNormal.x, mirrorNormal.y, mirrorNormal.z);
    Serial.printf("  Raw Angles: Base=%.2f, Tilt=%.2f\n", baseAngle - config.baseAngleOffset, tiltAngle - config.tiltAngleOffset); // Show raw angles before offset/constrain
    Serial.printf("  Set Angles: Base=%.1f, Tilt=%.1f\n", baseAngle, tiltAngle);


    // Actuate the servos
    setMirrorAngles(baseAngle, tiltAngle);
  }

private:
  // Use ESP32Servo objects
  Servo baseServo;
  Servo tiltServo;
  const uint8_t baseServoPin;
  const uint8_t tiltServoPin;

  // Pre-calculated vectors
  Vector heliostatPos;
  Vector targetDir;


  // Calculate base (azimuth) servo angle from mirror normal
  float calculateBaseAngle(const Vector &normal) {
    // atan2 gives angle in radians (-PI to +PI) relative to +X axis in the XY plane
    // We need to map this to the servo's range based on coordinate system and mounting
    // Example assumes Azimuth=0 is North (+Y) and Azimuth=90 is East (+X)
    float angleRad = atan2(normal.x, normal.y); // Note: atan2(X, Y) for angle from +Y (North)
    float angleDeg = angleRad * 180.0 / M_PI;   // Convert to degrees (-180 to +180)

    // Adjust angle based on calibration offset and constrain to servo limits
    float servoAngle = angleDeg + config.baseAngleOffset;

    // Handle angle wrap-around if needed (e.g., normalize to 0-360 or -180 to 180)
    // Simple constraint is often sufficient for servos
    return constrain(servoAngle, config.minServoAngle, config.maxServoAngle);
  }

  // Calculate tilt (elevation) servo angle from mirror normal
  float calculateTiltAngle(const Vector &normal) {
    // asin(normal.z) gives angle in radians (-PI/2 to +PI/2) between normal and XY plane
    // normal.z is sin(elevationAngle)
    float angleRad = asin(normal.z);
    float angleDeg = angleRad * 180.0 / M_PI; // Convert to degrees (-90 to +90)

    // Adjust angle based on calibration offset and constrain to servo limits
    // Example: If servo 0 is horizontal, 90 is up, offset maps [-90,90] to [0, 180]
    float servoAngle = angleDeg + config.tiltAngleOffset;
    return constrain(servoAngle, config.minServoAngle, config.maxServoAngle);
  }

  // Set servo angles (takes float, Servo library handles conversion to int)
  void setMirrorAngles(float baseAngle, float tiltAngle) {
    baseServo.write(baseAngle);
    tiltServo.write(tiltAngle);
  }
};

// --- Global Variables ---
// IMPORTANT: These need to be updated periodically (e.g., from RTC + sun position library, sensors, or Serial input)
// Units: Degrees. See coordinate system definition above.
float sunAzimuth = 90.0;  // Example: Sun in the East
float sunElevation = 30.0; // Example: Sun 30 degrees above horizon

// Heliostat object (using pins 5, 6 as example - check your board!)
// Common ESP32 GPIOs: 4, 5, 12, 13, 14, 15, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33
Heliostat heliostat(26, 25); // Example: Using GPIO26 for Base, GPIO25 for Tilt

// --- Arduino Sketch ---
void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for serial connection (optional)
  Serial.println("\n\nHeliostat Control Starting...");

  heliostat.init();
}

void loop() {
  // --- 1. Update Sun Position ---
  // In a real application, update sunAzimuth and sunElevation here.
  // For now, we use the fixed global values.
  // Example: Manually change sunAzimuth over time for testing
  // sunAzimuth += 1.0; if (sunAzimuth >= 360.0) sunAzimuth -= 360.0;

  Serial.printf("\n--- Update Cycle ---\n");
  Serial.printf("Current Sun Pos: Azimuth=%.1f deg, Elevation=%.1f deg\n", sunAzimuth, sunElevation);

  // --- 2. Convert Sun Angles to Direction Vector ---
  // Convert degrees to radians
  float sunAzimuthRad = radians(sunAzimuth);
  float sunElevationRad = radians(sunElevation);

  // Calculate sun direction vector based on coordinate system:
  // X = East, Y = North, Z = Up
  // Azimuth from North (+Y), positive East (+X)
  // Elevation from XY plane, positive Up (+Z)
  Vector sunDirection = {
      cos(sunElevationRad) * sin(sunAzimuthRad), // X component
      cos(sunElevationRad) * cos(sunAzimuthRad), // Y component
      sin(sunElevationRad)                       // Z component
  };
  sunDirection = sunDirection.normalize(); // Ensure it's a unit vector


  // --- 3. Update Heliostat ---
  heliostat.updateMirror(sunDirection);


  // --- 4. Wait for next update ---
  // Use delay for simplicity, or non-blocking millis() timer for advanced uses
  delay(5000); // Update every 5 seconds (adjust as needed)
}
