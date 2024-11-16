#include <Servo.h>
#include <math.h>

// Compile-time configuration for heliostat position offsets
#define X_OFFSET 1.0  // in meters
#define Y_OFFSET 2.0  // in meters
#define Z_OFFSET 0.5  // in meters

// Target position
#define TARGET_X 0.0  // in meters
#define TARGET_Y 0.0  // in meters
#define TARGET_Z 1.0  // in meters

// Global variables for sun position (to be updated by the base station)
float sunAzimuth = 0.0;   // in degrees
float sunElevation = 0.0; // in degrees

// Servo objects
Servo baseServo;
Servo tiltServo;

// Servo pin configuration
#define BASE_SERVO_PIN 5
#define TILT_SERVO_PIN 6

// Custom vector structure for algebraic geometry
struct Vector {
  float x, y, z;

  // Add two vectors
  Vector operator+(const Vector &v) const {
    return {x + v.x, y + v.y, z + v.z};
  }

  // Subtract two vectors
  Vector operator-(const Vector &v) const {
    return {x - v.x, y - v.y, z - v.z};
  }

  // Scale vector by a scalar
  Vector operator*(float scalar) const {
    return {x * scalar, y * scalar, z * scalar};
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

  // Normalize vector
  Vector normalize() const {
    float magnitude = sqrt(x * x + y * y + z * z);
    return {x / magnitude, y / magnitude, z / magnitude};
  }
};

// Function prototypes
void calculateHeliostatAngles(const Vector &sunDirection, const Vector &targetPosition, float &baseAngle, float &tiltAngle);

void setup() {
  // Attach servos
  baseServo.attach(BASE_SERVO_PIN);
  tiltServo.attach(TILT_SERVO_PIN);

  // Initial position
  baseServo.write(90);
  tiltServo.write(90);

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

  // Define the target position relative to heliostat
  Vector targetPosition = {TARGET_X - X_OFFSET, TARGET_Y - Y_OFFSET, TARGET_Z - Z_OFFSET};

  // Calculate angles to direct the mirror
  float baseAngle, tiltAngle;
  calculateHeliostatAngles(sunDirection, targetPosition, baseAngle, tiltAngle);

  // Update servo positions
  baseServo.write(baseAngle);
  tiltServo.write(tiltAngle);

  // Debug output
  Serial.print("Base Angle: ");
  Serial.print(baseAngle);
  Serial.print(" | Tilt Angle: ");
  Serial.println(tiltAngle);

  delay(1000); // Update interval
}

void calculateHeliostatAngles(const Vector &sunDirection, const Vector &targetPosition, float &baseAngle, float &tiltAngle) {
  // Normalize the target position vector
  Vector targetDir = targetPosition.normalize();

  // Calculate the mirror normal vector as the average of sunDirection and targetDir
  Vector mirrorNormal = (sunDirection + targetDir).normalize();

  // Base angle: Project mirror normal onto the X-Y plane and find its direction
  Vector mirrorNormalXY = {mirrorNormal.x, mirrorNormal.y, 0.0};
  baseAngle = atan2(mirrorNormalXY.y, mirrorNormalXY.x) * 180.0 / PI;

  // Tilt angle: Calculate the elevation of the mirror normal
  tiltAngle = asin(mirrorNormal.z) * 180.0 / PI;

  // Adjust angles for servo constraints (0-180 degrees)
  baseAngle = constrain(baseAngle + 90, 0, 180);
  tiltAngle = constrain(tiltAngle + 90, 0, 180);
}
