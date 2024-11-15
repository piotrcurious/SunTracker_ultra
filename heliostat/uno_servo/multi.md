 I'll modify the Arduino code to control multiple heliostats by using multiplexed servo control. This will allow one Arduino to manage several heliostats using a servo multiplexer setup.



<antArtifact identifier="arduino-heliostat-actuator" type="application/vnd.ant.code" language="cpp" title="Arduino Uno Multi-Heliostat Controller">
#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>
#include <PCA9685.h>  // PCA9685 servo driver

// Constants for PCA9685 servo driver
#define SERVO_FREQ 50
#define SERVOMIN  125 // Minimum pulse length count (out of 4096)
#define SERVOMAX  575 // Maximum pulse length count (out of 4096)

// System configuration
#define MAX_HELIOSTATS 8  // Maximum number of supported heliostats
#define SERIAL_BAUD 115200
#define COMMAND_BUFFER_SIZE 64
#define REGISTRATION_RETRY_DELAY 5000

// EEPROM configuration
#define EEPROM_CONFIG_START 0
#define EEPROM_CONFIG_SIZE sizeof(SystemConfig)

// Heliostat structure
struct Heliostat {
    int id;
    float x_offset;
    float y_offset;
    float z_offset;
    float current_azimuth;
    float current_elevation;
    uint8_t azimuth_channel;   // PCA9685 channel for azimuth servo
    uint8_t elevation_channel; // PCA9685 channel for elevation servo
    bool active;
};

// System configuration structure (stored in EEPROM)
struct SystemConfig {
    uint8_t num_heliostats;
    Heliostat heliostats[MAX_HELIOSTATS];
    uint32_t config_checksum;
};

// Global variables
PCA9685 servos;  // Servo driver instance
SystemConfig config;
char cmd_buffer[COMMAND_BUFFER_SIZE];
int cmd_index = 0;
bool command_complete = false;
unsigned long last_registration_attempt = 0;

// Calculate configuration checksum
uint32_t calculateChecksum() {
    uint32_t checksum = 0;
    uint8_t* ptr = (uint8_t*)&config;
    for (size_t i = 0; i < sizeof(SystemConfig) - sizeof(uint32_t); i++) {
        checksum += ptr[i];
    }
    return checksum;
}

// Load configuration from EEPROM
void loadConfiguration() {
    EEPROM.get(EEPROM_CONFIG_START, config);
    
    // Verify checksum
    uint32_t calculated_checksum = calculateChecksum();
    if (calculated_checksum != config.config_checksum) {
        // Invalid configuration, initialize defaults
        config.num_heliostats = 0;
        for (int i = 0; i < MAX_HELIOSTATS; i++) {
            config.heliostats[i].active = false;
        }
        saveConfiguration();
    }
    
    Serial.print("Loaded configuration with ");
    Serial.print(config.num_heliostats);
    Serial.println(" heliostats");
}

// Save configuration to EEPROM
void saveConfiguration() {
    config.config_checksum = calculateChecksum();
    EEPROM.put(EEPROM_CONFIG_START, config);
}

// Convert angle to servo pulse value
uint16_t angleToServoPulse(float angle) {
    // Constrain angle to 0-180 degrees
    angle = constrain(angle, 0, 180);
    // Map angle to pulse value
    return map(angle * 10, 0, 1800, SERVOMIN, SERVOMAX);
}

// Set servo position
void setServoPosition(uint8_t channel, float angle) {
    uint16_t pulse = angleToServoPulse(angle);
    servos.setPWM(channel, 0, pulse);
}

// Find heliostat by ID
Heliostat* findHeliostat(int id) {
    for (int i = 0; i < MAX_HELIOSTATS; i++) {
        if (config.heliostats[i].active && config.heliostats[i].id == id) {
            return &config.heliostats[i];
        }
    }
    return nullptr;
}

// Find next available servo channels
bool findAvailableChannels(uint8_t& az_channel, uint8_t& el_channel) {
    bool channels[16] = {false};  // Track used channels
    
    // Mark used channels
    for (int i = 0; i < MAX_HELIOSTATS; i++) {
        if (config.heliostats[i].active) {
            channels[config.heliostats[i].azimuth_channel] = true;
            channels[config.heliostats[i].elevation_channel] = true;
        }
    }
    
    // Find two consecutive free channels
    for (int i = 0; i < 15; i++) {
        if (!channels[i] && !channels[i + 1]) {
            az_channel = i;
            el_channel = i + 1;
            return true;
        }
    }
    return false;
}

// Add new heliostat
bool addHeliostat(int id, float x, float y, float z) {
    // Check if ID already exists
    if (findHeliostat(id) != nullptr) {
        Serial.println("ERR Heliostat ID already exists");
        return false;
    }
    
    // Find empty slot
    int slot = -1;
    for (int i = 0; i < MAX_HELIOSTATS; i++) {
        if (!config.heliostats[i].active) {
            slot = i;
            break;
        }
    }
    
    if (slot == -1) {
        Serial.println("ERR No free slots available");
        return false;
    }
    
    // Find available servo channels
    uint8_t az_channel, el_channel;
    if (!findAvailableChannels(az_channel, el_channel)) {
        Serial.println("ERR No available servo channels");
        return false;
    }
    
    // Initialize new heliostat
    config.heliostats[slot].id = id;
    config.heliostats[slot].x_offset = x;
    config.heliostats[slot].y_offset = y;
    config.heliostats[slot].z_offset = z;
    config.heliostats[slot].current_azimuth = 90.0;
    config.heliostats[slot].current_elevation = 45.0;
    config.heliostats[slot].azimuth_channel = az_channel;
    config.heliostats[slot].elevation_channel = el_channel;
    config.heliostats[slot].active = true;
    
    config.num_heliostats++;
    saveConfiguration();
    
    // Move to default position
    setServoPosition(az_channel, 90.0);
    setServoPosition(el_channel, 45.0);
    
    Serial.print("Added heliostat ID ");
    Serial.print(id);
    Serial.print(" at channels ");
    Serial.print(az_channel);
    Serial.print(",");
    Serial.println(el_channel);
    
    return true;
}

// Remove heliostat
bool removeHeliostat(int id) {
    for (int i = 0; i < MAX_HELIOSTATS; i++) {
        if (config.heliostats[i].active && config.heliostats[i].id == id) {
            // Move to safe position before deactivating
            setServoPosition(config.heliostats[i].azimuth_channel, 90.0);
            setServoPosition(config.heliostats[i].elevation_channel, 45.0);
            
            config.heliostats[i].active = false;
            config.num_heliostats--;
            saveConfiguration();
            
            Serial.print("Removed heliostat ID ");
            Serial.println(id);
            return true;
        }
    }
    Serial.println("ERR Heliostat not found");
    return false;
}

// Send registration for all active heliostats
void sendRegistrations() {
    for (int i = 0; i < MAX_HELIOSTATS; i++) {
        if (config.heliostats[i].active) {
            Serial.print("ADD ");
            Serial.print(config.heliostats[i].id);
            Serial.print(" ");
            Serial.print(config.heliostats[i].x_offset);
            Serial.print(" ");
            Serial.print(config.heliostats[i].y_offset);
            Serial.print(" ");
            Serial.println(config.heliostats[i].z_offset);
        }
    }
    last_registration_attempt = millis();
}

// Execute received command
void executeCommand(char* command) {
    char cmd[10];
    int id;
    float x, y, z, az, el;
    
    // Parse command
    if (sscanf(command, "%s", cmd) == 1) {
        if (strcmp(cmd, "ADD") == 0) {
            if (sscanf(command, "ADD %d %f %f %f", &id, &x, &y, &z) == 4) {
                addHeliostat(id, x, y, z);
            }
        }
        else if (strcmp(cmd, "REMOVE") == 0) {
            if (sscanf(command, "REMOVE %d", &id) == 1) {
                removeHeliostat(id);
            }
        }
        else if (strcmp(cmd, "HELI") == 0) {
            if (sscanf(command, "HELI %d AZ %f EL %f", &id, &az, &el) == 3) {
                Heliostat* heli = findHeliostat(id);
                if (heli != nullptr) {
                    setServoPosition(heli->azimuth_channel, az);
                    setServoPosition(heli->elevation_channel, el);
                    heli->current_azimuth = az;
                    heli->current_elevation = el;
                }
            }
        }
    }
}

void setup() {
    // Initialize serial communication
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
        ; // Wait for serial port to connect
    }
    
    // Initialize I2C
    Wire.begin();
    
    // Initialize servo driver
    servos.begin();
    servos.setPWMFreq(SERVO_FREQ);
    
    // Load configuration
    loadConfiguration();
    
    // Initial registration attempt
    sendRegistrations();
    
    Serial.println("Multi-heliostat controller ready");
}

void loop() {
    // Handle periodic registration attempts
    if (millis() - last_registration_attempt > REGISTRATION_RETRY_DELAY) {
        sendRegistrations();
    }
    
    // Read serial commands
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '\n') {
            cmd_buffer[cmd_index] = '\0';  // Null terminate
            command_complete = true;
            break;
        }
        else if (cmd_index < COMMAND_BUFFER_SIZE - 1) {
            cmd_buffer[cmd_index++] = c;
        }
    }
    
    // Execute command if complete
    if (command_complete) {
        executeCommand(cmd_buffer);
        
        // Reset buffer
        cmd_index = 0;
        command_complete = false;
    }
    
    // Small delay to prevent CPU hogging
    delay(10);
}
</antArtifact>

Key changes and features:

1. Added PCA9685 servo driver support:
   - Controls up to 16 servos via I2C
   - Each heliostat uses 2 channels (azimuth + elevation)
   - Maximum of 8 heliostats per Arduino (16 channels / 2)

2. Enhanced configuration management:
   - Stores all heliostat data in EEPROM
   - Checksum verification for configuration integrity
   - Automatic channel assignment for new heliostats

3. Multi-heliostat support:
   - Dynamic add/remove of heliostats
   - Each heliostat has unique ID and servo channels
   - Maintains position state for each heliostat

4. Improved command handling:
   - Supports ADD, REMOVE, and HELI commands
   - Each command affects specific heliostat by ID
   - Registration system for all active heliostats

Hardware requirements:
1. Arduino Uno
2. PCA9685 16-channel PWM/Servo driver
3. Power supply suitable for multiple servos
4. Servos (2 per heliostat)
5. I2C connection wiring

Wiring:
1. Arduino SDA -> PCA9685 SDA
2. Arduino SCL -> PCA9685 SCL
3. Arduino GND -> PCA9685 GND
4. 5V power supply -> PCA9685 V+
5. GND power supply -> PCA9685 GND
6. Servos connect to PCA9685 output channels

Would you like me to:
1. Add any additional features?
2. Explain any part in more detail?
3. Provide wiring diagrams?
4. Add error handling for specific scenarios?
