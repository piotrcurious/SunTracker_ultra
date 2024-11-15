#include <math.h>
#include <vector>

// Shared sun position variables (from solar position calculation system)
extern float sun_azimuth;   // Sun azimuth angle in degrees
extern float sun_elevation; // Sun elevation angle in degrees

// Heliostat structure for configuration and control
struct Heliostat {
    int id;          // Unique ID for the heliostat
    float x_offset;  // X-axis offset in meters
    float y_offset;  // Y-axis offset in meters
    float z_offset;  // Z-axis offset in meters
    float mirror_azimuth;    // Calculated azimuth for the mirror
    float mirror_elevation;  // Calculated elevation for the mirror
};

// Heliostat instances (stored in a vector for dynamic management)
std::vector<Heliostat> heliostats;

// Calculate mirror angles for a heliostat
void calculateMirrorAngles(Heliostat& heliostat) {
    // Convert sun angles to radians
    float sun_azimuth_rad = sun_azimuth * M_PI / 180.0;
    float sun_elevation_rad = sun_elevation * M_PI / 180.0;

    // Sun unit vector (normalized)
    float sun_x = cos(sun_elevation_rad) * cos(sun_azimuth_rad);
    float sun_y = cos(sun_elevation_rad) * sin(sun_azimuth_rad);
    float sun_z = sin(sun_elevation_rad);

    // Calculate heliostat mirror normal vector
    // Target vector is assumed to be (0, 0, 1) for directing sunlight vertically
    float target_x = 0.0;
    float target_y = 0.0;
    float target_z = 1.0;

    // Compute halfway vector
    float halfway_x = sun_x + target_x;
    float halfway_y = sun_y + target_y;
    float halfway_z = sun_z + target_z;
    float halfway_mag = sqrt(halfway_x * halfway_x + halfway_y * halfway_y + halfway_z * halfway_z);
    halfway_x /= halfway_mag;
    halfway_y /= halfway_mag;
    halfway_z /= halfway_mag;

    // Compute mirror angles (azimuth and elevation)
    heliostat.mirror_azimuth = atan2(halfway_y, halfway_x) * 180.0 / M_PI;
    heliostat.mirror_elevation = asin(halfway_z) * 180.0 / M_PI;
}

// Send actuator commands for a heliostat through serial port
void sendActuatorCommands(int heliostat_id, float azimuth, float elevation) {
    Serial.print("HELI ");
    Serial.print(heliostat_id);
    Serial.print(" AZ ");
    Serial.print(azimuth);
    Serial.print(" EL ");
    Serial.println(elevation);
}

// Update all heliostats
void updateHeliostats() {
    for (auto& heliostat : heliostats) {
        // Calculate mirror angles for each heliostat
        calculateMirrorAngles(heliostat);

        // Send actuator commands
        sendActuatorCommands(heliostat.id, heliostat.mirror_azimuth, heliostat.mirror_elevation);
    }
}

// Add a new heliostat dynamically
void addHeliostat(int id, float x_offset, float y_offset, float z_offset) {
    // Check if the heliostat ID already exists
    for (auto& heliostat : heliostats) {
        if (heliostat.id == id) {
            Serial.print("Heliostat ID ");
            Serial.print(id);
            Serial.println(" already exists!");
            return;
        }
    }

    // Add new heliostat
    Heliostat new_heliostat = {id, x_offset, y_offset, z_offset, 0.0, 0.0};
    heliostats.push_back(new_heliostat);
    Serial.print("Added heliostat ID ");
    Serial.println(id);
}

// Remove a heliostat dynamically
void removeHeliostat(int id) {
    for (auto it = heliostats.begin(); it != heliostats.end(); ++it) {
        if (it->id == id) {
            heliostats.erase(it);
            Serial.print("Removed heliostat ID ");
            Serial.println(id);
            return;
        }
    }
    Serial.print("Heliostat ID ");
    Serial.print(id);
    Serial.println(" not found!");
}

// Parse serial input for remote configuration commands
void parseSerialInput() {
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');
        command.trim();

        if (command.startsWith("ADD ")) {
            // Add heliostat: "ADD <id> <x_offset> <y_offset> <z_offset>"
            int id;
            float x_offset, y_offset, z_offset;
            if (sscanf(command.c_str(), "ADD %d %f %f %f", &id, &x_offset, &y_offset, &z_offset) == 4) {
                addHeliostat(id, x_offset, y_offset, z_offset);
            } else {
                Serial.println("Invalid ADD command format!");
            }
        } else if (command.startsWith("REMOVE ")) {
            // Remove heliostat: "REMOVE <id>"
            int id;
            if (sscanf(command.c_str(), "REMOVE %d", &id) == 1) {
                removeHeliostat(id);
            } else {
                Serial.println("Invalid REMOVE command format!");
            }
        } else {
            Serial.println("Unknown command!");
        }
    }
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);

    // Test: Set initial sun position (this would be updated in real-time by another system)
    sun_azimuth = 45.0;   // Example azimuth in degrees
    sun_elevation = 30.0; // Example elevation in degrees

    Serial.println("Heliostat control system ready.");
    Serial.println("Commands: ADD <id> <x_offset> <y_offset> <z_offset>, REMOVE <id>");
}

void loop() {
    // Check for incoming commands
    parseSerialInput();

    // Update heliostats periodically
    updateHeliostats();

    // Delay before next update (e.g., 1 second)
    delay(1000);
}
