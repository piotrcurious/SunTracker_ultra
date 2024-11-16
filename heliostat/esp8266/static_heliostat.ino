#include <Servo.h>

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
  // Calculate angles to direct the mirror
  float baseAngle, tiltAngle;
  calculateHeliostatAngles(sunAzimuth, sunElevation, baseAngle, tiltAngle);

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

// Function to calculate heliostat angles
void calculateHeliostatAngles(float sunAzimuth, float sunElevation, float &baseAngle, float &tiltAngle) {
  // Convert sun position to radians
  float sunAzimuthRad = radians(sunAzimuth);
  float sunElevationRad = radians(sunElevation);

  // Calculate sun direction vector
  float sunDirX = cos(sunElevationRad) * cos(sunAzimuthRad);
  float sunDirY = cos(sunElevationRad) * sin(sunAzimuthRad);
  float sunDirZ = sin(sunElevationRad);

  // Target vector from heliostat
  float targetDirX = TARGET_X - X_OFFSET;
  float targetDirY = TARGET_Y - Y_OFFSET;
  float targetDirZ = TARGET_Z - Z_OFFSET;

  // Normalize the target vector
  float targetMag = sqrt(sq(targetDirX) + sq(targetDirY) + sq(targetDirZ));
  targetDirX /= targetMag;
  targetDirY /= targetMag;
  targetDirZ /= targetMag;

  // Calculate mirror normal vector
  float mirrorNormalX = (sunDirX + targetDirX) / 2.0;
  float mirrorNormalY = (sunDirY + targetDirY) / 2.0;
  float mirrorNormalZ = (sunDirZ + targetDirZ) / 2.0;

  // Normalize the mirror normal vector
  float mirrorMag = sqrt(sq(mirrorNormalX) + sq(mirrorNormalY) + sq(mirrorNormalZ));
  mirrorNormalX /= mirrorMag;
  mirrorNormalY /= mirrorMag;
  mirrorNormalZ /= mirrorMag;

  // Calculate base and tilt angles from the mirror normal
  baseAngle = atan2(mirrorNormalY, mirrorNormalX) * 180.0 / PI;
  tiltAngle = asin(mirrorNormalZ) * 180.0 / PI;

  // Adjust angles for servo constraints (0-180 degrees)
  baseAngle = constrain(baseAngle + 90, 0, 180);
  tiltAngle = constrain(tiltAngle + 90, 0, 180);
}
