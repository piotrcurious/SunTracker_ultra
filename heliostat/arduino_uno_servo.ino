#include <Servo.h>

// Servo objects for azimuth and elevation control
Servo azimuthServo;
Servo elevationServo;

// Servo angles
int azimuthAngle = 90;   // Initial position for azimuth servo
int elevationAngle = 90; // Initial position for elevation servo

// Servo pin assignments
const int azimuthServoPin = 9;    // Pin connected to the azimuth servo
const int elevationServoPin = 10; // Pin connected to the elevation servo

// Heliostat parameters
const int heliostatID = 1;       // Unique ID for this heliostat
const float xOffset = 1.0;       // X-offset in meters
const float yOffset = 0.5;       // Y-offset in meters
const float zOffset = 0.2;       // Z-offset in meters
const int azimuthMin = 0;        // Minimum azimuth angle
const int azimuthMax = 180;      // Maximum azimuth angle
const int elevationMin = 0;      // Minimum elevation angle
const int elevationMax = 90;     // Maximum elevation angle

// Parse and execute incoming serial commands
void parseSerialInput(String command) {
    command.trim(); // Remove leading and trailing whitespace

    if (command.startsWith("HELI ")) {
        // Parse heliostat command: "HELI <id> AZ <azimuth> EL <elevation>"
        int id;
        float azimuth, elevation;
        if (sscanf(command.c_str(), "HELI %d AZ %f EL %f", &id, &azimuth, &elevation) == 3) {
            if (id == heliostatID) {
                Serial.print("Command for heliostat ID: ");
                Serial.println(id);

                // Convert azimuth and elevation to servo angles
                azimuthAngle = constrain(map(azimuth, 0, 180, azimuthMin, azimuthMax), azimuthMin, azimuthMax);
                elevationAngle = constrain(map(elevation, 0, 90, elevationMax, elevationMin), elevationMin, elevationMax);

                // Update servos
                azimuthServo.write(azimuthAngle);
                elevationServo.write(elevationAngle);

                Serial.print("Azimuth set to angle: ");
                Serial.println(azimuthAngle);
                Serial.print("Elevation set to angle: ");
                Serial.println(elevationAngle);
            } else {
                Serial.println("Heliostat ID does not match.");
            }
        } else {
            Serial.println("Invalid HELI command format!");
        }
    } else if (command == "REQUEST_PARAMS") {
        // Respond with heliostat parameters
        sendHeliostatParameters();
    } else {
        Serial.println("Unknown command received!");
    }
}

// Send heliostat parameters to the ESP8266
void sendHeliostatParameters() {
    Serial.print("PARAMS ID=");
    Serial.print(heliostatID);
    Serial.print(" X=");
    Serial.print(xOffset, 2);
    Serial.print(" Y=");
    Serial.print(yOffset, 2);
    Serial.print(" Z=");
    Serial.print(zOffset, 2);
    Serial.print(" AZ_MIN=");
    Serial.print(azimuthMin);
    Serial.print(" AZ_MAX=");
    Serial.print(azimuthMax);
    Serial.print(" EL_MIN=");
    Serial.print(elevationMin);
    Serial.print(" EL_MAX=");
    Serial.println(elevationMax);
}

void setup() {
    // Initialize serial communication
    Serial.begin(9600);

    // Attach servos to their respective pins
    azimuthServo.attach(azimuthServoPin);
    elevationServo.attach(elevationServoPin);

    // Initialize servos to their default positions
    azimuthServo.write(azimuthAngle);
    elevationServo.write(elevationAngle);

    // Send heliostat parameters upon startup
    sendHeliostatParameters();

    Serial.println("Heliostat device ready. Waiting for commands...");
}

void loop() {
    // Check if data is available on the serial port
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n'); // Read the incoming command
        parseSerialInput(command); // Process the command
    }

    // Delay to allow servos to stabilize after movement
    delay(100);
}
