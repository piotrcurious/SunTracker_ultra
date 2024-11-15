 I'll modify the code to simplify the EEPROM storage by only storing XYZ offsets and using sequential servo channel pairs for heliostats.



<antArtifact identifier="heliostat-controller" type="application/vnd.ant.code" language="cpp" title="Heliostat Controller with Simplified EEPROM Storage">
#include <Wire.h>
#include <Servo.h>
#include <EEPROM.h>
#include <PCA9685.h>
#include <IRremote.h>

// Constants for PCA9685 servo driver
#define SERVO_FREQ 50
#define SERVOMIN  125
#define SERVOMAX  575

// System configuration
#define MAX_HELIOSTATS 8
#define SERIAL_BAUD 115200
#define COMMAND_BUFFER_SIZE 64
#define REGISTRATION_RETRY_DELAY 5000
#define IR_RECEIVE_PIN 2

// IR Remote Commands (RC5 Protocol)
#define IR_CMD_SELECT 0x01
#define IR_CMD_SET_X 0x02
#define IR_CMD_SET_Y 0x03
#define IR_CMD_SET_Z 0x04
#define IR_CMD_ADJUST_UP 0x05
#define IR_CMD_ADJUST_DOWN 0x06
#define IR_CMD_SAVE 0x07

// Adjustment values
#define OFFSET_ADJUSTMENT_STEP 0.1  // Meters to adjust per button press

// Simplified heliostat structure - only stores offsets
struct HeliostatOffsets {
    float x;
    float y;
    float z;
    bool active;
};

// EEPROM configuration
#define EEPROM_CONFIG_START 0
#define EEPROM_MAGIC_NUMBER 0xAB12  // Used to verify EEPROM initialization

struct EEPROMConfig {
    uint16_t magic;  // Magic number to verify EEPROM state
    HeliostatOffsets heliostats[MAX_HELIOSTATS];
};

// Runtime heliostat tracking
struct RuntimeHeliostat {
    float current_azimuth;
    float current_elevation;
};

// IR control state
struct IRControlState {
    bool config_mode;
    int selected_heliostat;
    uint8_t adjusting_axis;  // 0=X, 1=Y, 2=Z
    unsigned long last_ir_command;
    float temp_x_offset;
    float temp_y_offset;
    float temp_z_offset;
} ir_state;

// Global variables
PCA9685 servos;
EEPROMConfig config;
RuntimeHeliostat runtime_heliostats[MAX_HELIOSTATS];
char cmd_buffer[COMMAND_BUFFER_SIZE];
int cmd_index = 0;
bool command_complete = false;
unsigned long last_registration_attempt = 0;
IRrecv irrecv(IR_RECEIVE_PIN);
decode_results ir_results;

// Get servo channels for heliostat
void getHeliostatChannels(int heliostat_index, uint8_t& az_channel, uint8_t& el_channel) {
    az_channel = heliostat_index * 2;
    el_channel = az_channel + 1;
}

// Load configuration from EEPROM
void loadConfiguration() {
    EEPROM.get(EEPROM_CONFIG_START, config);
    
    // Check if EEPROM is initialized
    if (config.magic != EEPROM_MAGIC_NUMBER) {
        // Initialize with defaults
        config.magic = EEPROM_MAGIC_NUMBER;
        for (int i = 0; i < MAX_HELIOSTATS; i++) {
            config.heliostats[i].active = false;
            config.heliostats[i].x = 0;
            config.heliostats[i].y = 0;
            config.heliostats[i].z = 0;
        }
        saveConfiguration();
    }
    
    // Initialize runtime states
    for (int i = 0; i < MAX_HELIOSTATS; i++) {
        runtime_heliostats[i].current_azimuth = 90.0;
        runtime_heliostats[i].current_elevation = 45.0;
    }
    
    Serial.println("Configuration loaded");
}

// Save configuration to EEPROM
void saveConfiguration() {
    EEPROM.put(EEPROM_CONFIG_START, config);
}

// Convert angle to servo pulse value
uint16_t angleToServoPulse(float angle) {
    angle = constrain(angle, 0, 180);
    return map(angle * 10, 0, 1800, SERVOMIN, SERVOMAX);
}

// Set servo position
void setServoPosition(uint8_t channel, float angle) {
    uint16_t pulse = angleToServoPulse(angle);
    servos.setPWM(channel, 0, pulse);
}

// Handle IR commands
void handleIRCommand(unsigned long command) {
    ir_state.last_ir_command = millis();
    
    switch(command) {
        case IR_CMD_SELECT: {
            delay(500);
            if (irrecv.decode(&ir_results)) {
                int heliostat_num = ir_results.value & 0x0F;
                if (heliostat_num < MAX_HELIOSTATS) {
                    ir_state.selected_heliostat = heliostat_num;
                    ir_state.config_mode = true;
                    ir_state.adjusting_axis = 0;
                    ir_state.temp_x_offset = config.heliostats[heliostat_num].x;
                    ir_state.temp_y_offset = config.heliostats[heliostat_num].y;
                    ir_state.temp_z_offset = config.heliostats[heliostat_num].z;
                    config.heliostats[heliostat_num].active = true;
                    Serial.print("Selected heliostat ");
                    Serial.println(heliostat_num);
                    printCurrentOffsets();
                }
                irrecv.resume();
            }
            break;
        }
        
        case IR_CMD_SET_X:
            if (ir_state.config_mode) {
                ir_state.adjusting_axis = 0;
                Serial.println("Adjusting X offset");
            }
            break;
            
        case IR_CMD_SET_Y:
            if (ir_state.config_mode) {
                ir_state.adjusting_axis = 1;
                Serial.println("Adjusting Y offset");
            }
            break;
            
        case IR_CMD_SET_Z:
            if (ir_state.config_mode) {
                ir_state.adjusting_axis = 2;
                Serial.println("Adjusting Z offset");
            }
            break;
            
        case IR_CMD_ADJUST_UP:
            if (ir_state.config_mode) {
                adjustCurrentOffset(OFFSET_ADJUSTMENT_STEP);
            }
            break;
            
        case IR_CMD_ADJUST_DOWN:
            if (ir_state.config_mode) {
                adjustCurrentOffset(-OFFSET_ADJUSTMENT_STEP);
            }
            break;
            
        case IR_CMD_SAVE:
            if (ir_state.config_mode) {
                saveHeliostatOffsets();
                ir_state.config_mode = false;
                Serial.println("Offsets saved");
            }
            break;
    }
}

// Adjust current offset
void adjustCurrentOffset(float adjustment) {
    switch(ir_state.adjusting_axis) {
        case 0:
            ir_state.temp_x_offset += adjustment;
            break;
        case 1:
            ir_state.temp_y_offset += adjustment;
            break;
        case 2:
            ir_state.temp_z_offset += adjustment;
            break;
    }
    printCurrentOffsets();
}

// Print current offsets
void printCurrentOffsets() {
    Serial.println("Current offsets:");
    Serial.print("X: "); Serial.print(ir_state.temp_x_offset, 2);
    Serial.print(" Y: "); Serial.print(ir_state.temp_y_offset, 2);
    Serial.print(" Z: "); Serial.println(ir_state.temp_z_offset, 2);
}

// Save heliostat offsets
void saveHeliostatOffsets() {
    if (ir_state.selected_heliostat >= 0 && ir_state.selected_heliostat < MAX_HELIOSTATS) {
        config.heliostats[ir_state.selected_heliostat].x = ir_state.temp_x_offset;
        config.heliostats[ir_state.selected_heliostat].y = ir_state.temp_y_offset;
        config.heliostats[ir_state.selected_heliostat].z = ir_state.temp_z_offset;
        saveConfiguration();
        
        // Send registration update
        Serial.print("ADD ");
        Serial.print(ir_state.selected_heliostat);
        Serial.print(" ");
        Serial.print(ir_state.temp_x_offset);
        Serial.print(" ");
        Serial.print(ir_state.temp_y_offset);
        Serial.print(" ");
        Serial.println(ir_state.temp_z_offset);
    }
}

// Send registrations for all active heliostats
void sendRegistrations() {
    for (int i = 0; i < MAX_HELIOSTATS; i++) {
        if (config.heliostats[i].active) {
            Serial.print("ADD ");
            Serial.print(i);
            Serial.print(" ");
            Serial.print(config.heliostats[i].x);
            Serial.print(" ");
            Serial.print(config.heliostats[i].y);
            Serial.print(" ");
            Serial.println(config.heliostats[i].z);
        }
    }
    last_registration_attempt = millis();
}

// Execute received command
void executeCommand(char* command) {
    char cmd[10];
    int id;
    float az, el;
    
    if (sscanf(command, "%s", cmd) == 1) {
        if (strcmp(cmd, "HELI") == 0) {
            if (sscanf(command, "HELI %d AZ %f EL %f", &id, &az, &el) == 3) {
                if (id < MAX_HELIOSTATS && config.heliostats[id].active) {
                    uint8_t az_channel, el_channel;
                    getHeliostatChannels(id, az_channel, el_channel);
                    
                    setServoPosition(az_channel, az);
                    setServoPosition(el_channel, el);
                    runtime_heliostats[id].current_azimuth = az;
                    runtime_heliostats[id].current_elevation = el;
                }
            }
        }
    }
}

void setup() {
    Serial.begin(SERIAL_BAUD);
    while (!Serial) {
        ; // Wait for serial port to connect
    }
    
    Wire.begin();
    servos.begin();
    servos.setPWMFreq(SERVO_FREQ);
    irrecv.enableIRIn();
    
    // Initialize IR control state
    ir_state.config_mode = false;
    ir_state.selected_heliostat = -1;
    ir_state.adjusting_axis = 0;
    ir_state.last_ir_command = 0;
    
    loadConfiguration();
    sendRegistrations();
    
    Serial.println("Multi-heliostat controller ready");
}

void loop() {
    if (irrecv.decode(&ir_results)) {
        handleIRCommand(ir_results.value);
        irrecv.resume();
    }
    
    if (ir_state.config_mode && (millis() - ir_state.last_ir_command > 30000)) {
        ir_state.config_mode = false;
        Serial.println("Config mode timeout");
    }
    
    if (millis() - last_registration_attempt > REGISTRATION_RETRY_DELAY) {
        sendRegistrations();
    }
    
    while (Serial.available() > 0) {
        char c = Serial.read();
        
        if (c == '\n') {
            cmd_buffer[cmd_index] = '\0';
            command_complete = true;
            break;
        }
        else if (cmd_index < COMMAND_BUFFER_SIZE - 1) {
            cmd_buffer[cmd_index++] = c;
        }
    }
    
    if (command_complete) {
        executeCommand(cmd_buffer);
        cmd_index = 0;
        command_complete = false;
    }
    
    delay(10);
}
</antArtifact>

Key improvements made to the code:

1. Simplified data structures:
   - Removed redundant fields from heliostat structure
   - Only storing XYZ offsets and active status in EEPROM
   - Added separate runtime structure for tracking current angles

2. Simplified servo channel allocation:
   - Automatic sequential channel assignment (heliostat 0 = channels 0,1; heliostat 1 = channels 2,3; etc.)
   - Added helper function `getHeliostatChannels()`

3. Improved EEPROM handling:
   - Added magic number validation
   - Reduced EEPROM writes by only storing essential data
   - Automatic initialization of EEPROM on first use

4. Other improvements:
   - Simplified heliostat identification (using array index as ID)
   - Removed unnecessary configuration parameters
   - Cleaner separation between stored and runtime data

To use the system:
1. Connect IR receiver to pin 2
2. Each heliostat uses two consecutive servo channels
3. Use RC5 remote to configure heliostat offsets:
   - SELECT + number (0-7) to choose heliostat
   - SET_X/Y/Z to select axis
   - UP/DOWN to adjust selected offset
   - SAVE to store configuration

The servos are now automatically assigned in pairs:
- Heliostat 0: channels 0 (azimuth) and 1 (elevation)
- Heliostat 1: channels 2 (azimuth) and 3 (elevation)
- And so on...

Would you like me to make any adjustments to this implementation?
