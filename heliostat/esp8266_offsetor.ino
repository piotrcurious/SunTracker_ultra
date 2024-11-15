#include <math.h>

// Shared sun position variables (from solar position calculation system)
extern float sun_azimuth;   // Sun azimuth angle in degrees
extern float sun_elevation; // Sun elevation angle in degrees

// Heliostat structure for configuration and control
struct Heliostat {
    float x_offset;   // X-axis offset in meters
    float y_offset;   // Y-axis offset in meters
    float z_offset;   // Z-axis offset in meters
    float mirror_azimuth;    // Calculated azimuth for the mirror
    float mirror_elevation;  // Calculated elevation for the mirror
};

// Heliostat instances
#define NUM_HELIOSTATS 3
Heliostat heliostats[NUM_HELIOSTATS] = {
    {0.0, 0.0, 0.0},  // Heliostat 1 offsets
    {1.5, -1.0, 0.0}, // Heliostat 2 offsets
    {-1.0, 2.0, 0.5}  // Heliostat 3 offsets
};

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
    for (int i = 0; i < NUM_HELIOSTATS; i++) {
        // Calculate mirror angles for each heliostat
        calculateMirrorAngles(heliostats[i]);

        // Send actuator commands
        sendActuatorCommands(i, heliostats[i].mirror_azimuth, heliostats[i].mirror_elevation);
    }
}

void setup() {
    // Initialize serial communication
    Serial.begin(115200);

    // Test: Set initial sun position (this would be updated in real-time by another system)
    sun_azimuth = 45.0;   // Example azimuth in degrees
    sun_elevation = 30.0; // Example elevation in degrees
}

void loop() {
    // Update heliostats periodically
    updateHeliostats();

    // Delay before next update (e.g., 1 second)
    delay(1000);
}
