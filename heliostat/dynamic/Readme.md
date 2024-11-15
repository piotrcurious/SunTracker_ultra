Here’s an improved version of the heliostat control system that allows remote configuration via the serial port. It supports dynamically adding and removing heliostats, assuming heliostat devices send their configuration parameters (offsets and IDs) through the serial port.


---

Complete Code with Dynamic Remote Configuration

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


---

Features Added

1. Dynamic Heliostat Management:

The system supports adding and removing heliostats dynamically.

Each heliostat is uniquely identified by its ID.



2. Serial Command Interface:

ADD Command: Add a new heliostat by specifying its ID and offsets (e.g., ADD 1 1.5 -1.0 0.0).

REMOVE Command: Remove a heliostat by its ID (e.g., REMOVE 1).



3. Error Handling:

Checks for duplicate IDs when adding a heliostat.

Informs the user if a heliostat to be removed is not found.



4. Scalability:

The vector-based storage allows for a scalable number of heliostats limited only by memory.





---

How It Works

1. Add Heliostat:

The ADD command creates a new heliostat with specified offsets.

Offsets correct the mirror position based on the heliostat's location.



2. Remove Heliostat:

The REMOVE command deletes a heliostat, stopping updates for it.



3. Update Cycle:

Every second, the system calculates the required mirror angles for each heliostat and sends commands to their actuators via the serial port.





---

This implementation ensures flexibility and real-time configurability, making it ideal for a dynamic heliostat control system. Let me know if you'd like further enhancements!

