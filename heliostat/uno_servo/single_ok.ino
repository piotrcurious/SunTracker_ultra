#include <Servo.h>
#include <string.h>
#include <EEPROM.h>

// Pin definitions for servos
#define AZIMUTH_SERVO_PIN 9   // Azimuth servo on pin 9
#define ELEVATION_SERVO_PIN 10 // Elevation servo on pin 10

// EEPROM addresses for stored configuration
#define EEPROM_ID_ADDR 0
#define EEPROM_X_ADDR 4
#define EEPROM_Y_ADDR 8
#define EEPROM_Z_ADDR 12
#define EEPROM_INITIALIZED_ADDR 16

// Constants
#define SERIAL_BAUD 115200
#define COMMAND_BUFFER_SIZE 64
#define SERVO_MIN_PULSE 544    // Minimum pulse width in microseconds
#define SERVO_MAX_PULSE 2400   // Maximum pulse width in microseconds
#define REGISTRATION_RETRY_DELAY 5000  // 5 seconds between registration attempts

// Heliostat configuration
struct HeliostatConfig {
    int id;
    float x_offset;
    float y_offset;
    float z_offset;
    bool is_initialized;
} config;

// Current state
float current_azimuth = 90.0;   // Current azimuth angle
float current_elevation = 45.0;  // Current elevation angle
unsigned long last_registration_attempt = 0;

// Servo objects
Servo azimuth_servo;    // Servo for azimuth control
Servo elevation_servo;  // Servo for elevation control

// Command parsing variables
char cmd_buffer[COMMAND_BUFFER_SIZE];
int cmd_index = 0;
bool command_complete = false;

// Function to load configuration from EEPROM
void loadConfiguration() {
    EEPROM.get(EEPROM_INITIALIZED_ADDR, config.is_initialized);
    if (config.is_initialized) {
        EEPROM.get(EEPROM_ID_ADDR, config.id);
        EEPROM.get(EEPROM_X_ADDR, config.x_offset);
        EEPROM.get(EEPROM_Y_ADDR, config.y_offset);
        EEPROM.get(EEPROM_Z_ADDR, config.z_offset);
        Serial.println("Configuration loaded from EEPROM");
    } else {
        // Default configuration for new devices
        config.id = -1;
        config.x_offset = 0.0;
        config.y_offset = 0.0;
        config.z_offset = 0.0;
        Serial.println("No stored configuration found");
    }
}

// Function to save configuration to EEPROM
void saveConfiguration() {
    config.is_initialized = true;
    EEPROM.put(EEPROM_INITIALIZED_ADDR, config.is_initialized);
    EEPROM.put(EEPROM_ID_ADDR, config.id);
    EEPROM.put(EEPROM_X_ADDR, config.x_offset);
    EEPROM.put(EEPROM_Y_ADDR, config.y_offset);
    EEPROM.put(EEPROM_Z_ADDR, config.z_offset);
}

// Function to send registration request
void sendRegistrationRequest() {
    // Send ADD command according to ESP8266 protocol
    Serial.print("ADD ");
    if (config.is_initialized) {
        // Send stored ID and position for re-registration
        Serial.print(config.id);
        Serial.print(" ");
    } else {
        // Request new ID (using -1 to indicate new device)
        Serial.print("-1 ");
    }
    // Send position (either stored or default 0,0,0)
    Serial.print(config.x_offset);
    Serial.print(" ");
    Serial.print(config.y_offset);
    Serial.print(" ");
    Serial.println(config.z_offset);
    
    last_registration_attempt = millis();
}

// Function to convert angles to servo positions
int angleToServo(float angle, bool is_elevation) {
    // Constrain angles to valid ranges
    if (is_elevation) {
        angle = constrain(angle, 0, 90);  // Elevation: 0° to 90°
    } else {
        angle = constrain(angle, 0, 180); // Azimuth: 0° to 180°
    }
    
    // Map angle to servo pulse width
    return map(angle * 10, 0, 1800, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
}

// Function to move servos smoothly to target position
void moveToPosition(float target_azimuth, float target_elevation) {
    // Calculate servo positions
    int az_pos = angleToServo(target_azimuth, false);
    int el_pos = angleToServo(target_elevation, true);
    
    // Set servo positions
    azimuth_servo.writeMicroseconds(az_pos);
    elevation_servo.writeMicroseconds(el_pos);
    
    // Update current positions
    current_azimuth = target_azimuth;
    current_elevation = target_elevation;
}

// Parse and execute received command
void executeCommand(char* command) {
    int id;
    float az, el;
    
    // Check for HELI command (format: "HELI <id> AZ <angle> EL <angle>")
    if (sscanf(command, "HELI %d AZ %f EL %f", &id, &az, &el) == 3) {
        // Only process if ID matches
        if (id == config.id) {
            moveToPosition(az, el);
        }
    }
}

void setup() {
    // Initialize serial communication
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
        ; // Wait for serial port to connect
    }
    
    // Attach and initialize servos
    azimuth_servo.attach(AZIMUTH_SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    elevation_servo.attach(ELEVATION_SERVO_PIN, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    
    // Load configuration from EEPROM
    loadConfiguration();
    
    // Move to default position
    moveToPosition(90.0, 45.0);
    
    // Initial registration attempt
    sendRegistrationRequest();
    
    Serial.println("Heliostat initializing...");
}

void loop() {
    // Handle periodic registration attempts
    if (millis() - last_registration_attempt > REGISTRATION_RETRY_DELAY) {
        sendRegistrationRequest();
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
