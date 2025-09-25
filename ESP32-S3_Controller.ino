#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ESP-NOW Communication Structure (must match drone code)
typedef struct {
    uint8_t command;        // 1=ARM, 2=DISARM, 3=THROTTLE, 4=ATTITUDE, 5=EMERGENCY
    int16_t throttle;       // 0-255 PWM value
    float roll_setpoint;    // Roll angle setpoint (-30 to +30 degrees)
    float pitch_setpoint;   // Pitch angle setpoint (-30 to +30 degrees)
    float yaw_setpoint;     // Yaw rate setpoint (-100 to +100 deg/s)
    uint32_t timestamp;     // For timeout detection
} esp_now_data_t;

// ESP-NOW Response Structure (drone status)
typedef struct {
    bool armed;
    int16_t throttle;
    float roll;
    float pitch;
    float battery_voltage;
    uint8_t signal_strength;
    uint32_t timestamp;
} esp_now_response_t;

esp_now_data_t sendData;
esp_now_response_t receivedStatus;
bool status_received = false;

// Drone MAC address - UPDATE THIS WITH YOUR DRONE'S MAC
uint8_t droneMAC[] = {0xec, 0xda, 0x3b, 0xe3, 0x0d, 0x7c};

// Hardware pin definitions for your ESP32-S3
#define THROTTLE_PIN 13     // GP13 - VRY joystick (throttle control)
#define YAW_PIN 12          // GP12 - VRX joystick (yaw control)
#define ARM_BUTTON_PIN 11   // GP11 - Push button to toggle ARM/DISARM
#define PITCH_PIN 10
#define ROLL_PIN 9
#define ARM_LED_PIN 48      // GP48 - LED to show ARM status

// Throttle/Yaw joystick (left stick)
#define THROTTLE_MIN_ADC   0
#define THROTTLE_MAX_ADC   4095
#define YAW_MIN_ADC        0
#define YAW_MAX_ADC        4095
#define YAW_CENTER_ADC     1865
#define YAW_DEADZONE       100

// Pitch/Roll joystick (right stick)
#define PITCH_MIN_ADC      0
#define PITCH_MAX_ADC      4095
#define PITCH_CENTER_ADC   1997
#define ROLL_MIN_ADC       0
#define ROLL_MAX_ADC       4095
#define ROLL_CENTER_ADC    1919
#define PITCHROLL_DEADZONE 100   // optional: prevents drift

// Control parameters
#define THROTTLE_MIN 0
#define THROTTLE_MAX 255
#define YAW_RATE_MAX 100.0  // Maximum yaw rate in deg/s

// Control variables
int current_throttle = 0;
float current_roll = 0.0;
float current_pitch = 0.0;
float current_yaw = 0.0;
bool controller_armed = false;  // Controller's view of arm state
bool drone_confirmed_armed = false;  // Drone's confirmed arm state

// Timing variables
unsigned long lastControlUpdate = 0;
unsigned long lastStatusRequest = 0;
unsigned long lastSyncCheck = 0;
unsigned long lastArmAttempt = 0;
const unsigned long CONTROL_UPDATE_INTERVAL = 50;  // 20Hz control updates
const unsigned long STATUS_REQUEST_INTERVAL = 500; // 2Hz status requests
const unsigned long SYNC_CHECK_INTERVAL = 1000;    // 1Hz sync check
const unsigned long ARM_RETRY_DELAY = 1000;        // 1 second between ARM attempts

// Button state tracking for toggle functionality
bool last_button_state = true;  // Start with button not pressed (pullup = HIGH)
unsigned long last_button_time = 0;
const unsigned long DEBOUNCE_DELAY = 50;  // 50ms debounce

// Communication tracking
unsigned long lastDroneResponse = 0;
bool communication_active = false;

void setup() {
    Serial.begin(115200);
    
    // Initialize pins
    pinMode(ARM_BUTTON_PIN, INPUT_PULLUP);  // Toggle button with pullup
    pinMode(ARM_LED_PIN, OUTPUT);           // ARM status LED
    
    // Start with LED OFF (disarmed state)
    digitalWrite(ARM_LED_PIN, LOW);
    
    Serial.println("ESP32-S3 Drone Controller Starting...");
    Serial.println("Hardware: Dual-Axis Joystick + Toggle Button + ARM LED");
    Serial.println("Enhanced with ARM synchronization and visual status");
    
    // Initialize ESP-NOW
    initESPNow();
    
    Serial.println("\n=== ESP32-S3 Drone Controller Ready ===");
    Serial.println("Hardware Controls:");
    Serial.println("  VRY Joystick (GP13) - Throttle control (0-255)");
    Serial.println("  VRX Joystick (GP12) - Yaw control (-100 to +100 deg/s)");
    Serial.println("  Toggle Button (GP11) - Press to ARM/DISARM drone");
    Serial.println("  ARM LED (GP48) - Shows ARM status (ON=ARMED, OFF=DISARMED)");
    Serial.println();
    Serial.println("Serial Commands:");
    Serial.println("  r <angle>  - Set roll angle (-30 to +30)");
    Serial.println("  p <angle>  - Set pitch angle (-30 to +30)");
    Serial.println("  center     - Level attitude (roll=0, pitch=0)");
    Serial.println("  status     - Show detailed status");
    Serial.println("  sync       - Force ARM/DISARM sync with drone");
    Serial.println("  check      - Preflight check");
    Serial.println("  cal        - Calibrate joystick range");
    Serial.println("  help       - Show this help");
    Serial.println("==========================================\n");
}

void loop() {
    unsigned long currentTime = millis();
    
    // Handle joystick and button inputs at 20Hz
    if (currentTime - lastControlUpdate >= CONTROL_UPDATE_INTERVAL) {
        readControlInputs();
        lastControlUpdate = currentTime;
    }
    
    // Handle serial commands
    handleSerialCommands();
    
    // Send periodic status requests
    if (currentTime - lastStatusRequest >= STATUS_REQUEST_INTERVAL) {
        requestStatus();
        lastStatusRequest = currentTime;
    }
    
    // Check ARM synchronization
    if (currentTime - lastSyncCheck >= SYNC_CHECK_INTERVAL) {
        checkArmSync();
        lastSyncCheck = currentTime;
    }
    
    // Update communication status
    updateCommunicationStatus();
    
    // Update ARM LED status
    updateArmLED();
}

void updateArmLED() {
    // LED indicates the synchronized ARM state
    if (controller_armed && drone_confirmed_armed) {
        digitalWrite(ARM_LED_PIN, HIGH);  // Solid ON = Both systems armed
    } else {
        digitalWrite(ARM_LED_PIN, LOW);   // OFF = Either system disarmed
    }
}

void readControlInputs() {
    // Read throttle and yaw joystick
    int throttle_raw = analogRead(THROTTLE_PIN);
    int yaw_raw = analogRead(YAW_PIN);

    // Read pitch and roll joystick (new!)
    int pitch_raw = analogRead(PITCH_PIN);
    int roll_raw  = analogRead(ROLL_PIN);

    // --- ARM button handling (same as before) ---
    bool current_button_state = digitalRead(ARM_BUTTON_PIN);
    unsigned long currentTime = millis();
    if (current_button_state != last_button_state && (currentTime - last_button_time) > DEBOUNCE_DELAY) {
        if (current_button_state == LOW) {
            toggleArmState(throttle_raw);
        }
        last_button_state = current_button_state;
        last_button_time = currentTime;
    }

    // --- Throttle mapping (0–255) ---
    int new_throttle = map(throttle_raw, THROTTLE_MIN_ADC, THROTTLE_MAX_ADC, THROTTLE_MIN, THROTTLE_MAX);
    new_throttle = constrain(new_throttle, THROTTLE_MIN, THROTTLE_MAX);

    // --- Yaw mapping with deadzone ---
    float new_yaw = 0.0;
    if (abs(yaw_raw - YAW_CENTER_ADC) > YAW_DEADZONE) {
        new_yaw = ((float)(yaw_raw - YAW_CENTER_ADC) / ((YAW_MAX_ADC - YAW_MIN_ADC) / 2.0)) * YAW_RATE_MAX;
        new_yaw = constrain(new_yaw, -YAW_RATE_MAX, YAW_RATE_MAX);
    }

    // --- Pitch mapping (-30° to +30°) ---
    float new_pitch = 0.0;
    if (abs(pitch_raw - PITCH_CENTER_ADC) > PITCHROLL_DEADZONE) {
        new_pitch = ((float)(pitch_raw - PITCH_CENTER_ADC) / ((PITCH_MAX_ADC - PITCH_MIN_ADC) / 2.0)) * 30.0;
        new_pitch = constrain(new_pitch, -30.0, 30.0);
    }

    // --- Roll mapping (-30° to +30°) ---
    float new_roll = 0.0;
    if (abs(roll_raw - ROLL_CENTER_ADC) > PITCHROLL_DEADZONE) {
        new_roll = ((float)(roll_raw - ROLL_CENTER_ADC) / ((ROLL_MAX_ADC - ROLL_MIN_ADC) / 2.0)) * 30.0;
        new_roll = constrain(new_roll, -30.0, 30.0);
    }

    // Debug every second
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 1000) {
        Serial.printf("JOY RAW: T=%d, Y=%d, P=%d, R=%d | MAP: T=%d, Y=%.1f, P=%.1f°, R=%.1f°\n",
                      throttle_raw, yaw_raw, pitch_raw, roll_raw,
                      new_throttle, new_yaw, new_pitch, new_roll);
        lastDebug = millis();
    }

    // --- Update active controls only when armed ---
    if (controller_armed && drone_confirmed_armed) {
        current_throttle = new_throttle;
        current_yaw = new_yaw;
        current_pitch = new_pitch;
        current_roll = new_roll;

        // Send command with all 4 controls
        sendCommand(4, current_throttle, current_roll, current_pitch, current_yaw);
    } else {
        current_throttle = 0;
        current_yaw = 0.0;
        current_pitch = 0.0;
        current_roll = 0.0;
    }
}

void calibrateJoystick() {
    Serial.println("\n=== Joystick Calibration ===");
    Serial.println("This will help find your joystick's actual range and center");
    Serial.println();
    
    Serial.println("Step 1: Move yaw joystick (GP12) FULL LEFT and hold for 3 seconds...");
    delay(3000);
    int yaw_min = analogRead(YAW_PIN);
    Serial.printf("Yaw minimum: %d\n", yaw_min);
    
    Serial.println("Step 2: Move yaw joystick FULL RIGHT and hold for 3 seconds...");
    delay(3000);
    int yaw_max = analogRead(YAW_PIN);
    Serial.printf("Yaw maximum: %d\n", yaw_max);
    
    Serial.println("Step 3: Move yaw joystick to CENTER and hold for 3 seconds...");
    delay(3000);
    int yaw_center = analogRead(YAW_PIN);
    Serial.printf("Yaw center: %d\n", yaw_center);
    
    Serial.println("Step 4: Move throttle joystick (GP13) to BOTTOM and hold for 3 seconds...");
    delay(3000);
    int throttle_min = analogRead(THROTTLE_PIN);
    Serial.printf("Throttle minimum: %d\n", throttle_min);
    
    Serial.println("Step 5: Move throttle joystick to TOP and hold for 3 seconds...");
    delay(3000);
    int throttle_max = analogRead(THROTTLE_PIN);
    Serial.printf("Throttle maximum: %d\n", throttle_max);
    
    Serial.println("\n=== Calibration Results ===");
    Serial.printf("Yaw range: %d to %d (center: %d)\n", yaw_min, yaw_max, yaw_center);
    Serial.printf("Throttle range: %d to %d\n", throttle_min, throttle_max);
    Serial.println();
    Serial.println("Update your code with these values:");
    Serial.printf("#define JOYSTICK_MIN %d\n", min(yaw_min, throttle_min));
    Serial.printf("#define JOYSTICK_MAX %d\n", max(yaw_max, throttle_max));
    Serial.printf("#define JOYSTICK_CENTER %d\n", yaw_center);
    Serial.println();
    Serial.println("Or if ranges are very different, use separate ranges for each axis.");
    Serial.println("============================\n");
}

void toggleArmState(int throttle_raw) {
    unsigned long currentTime = millis();
    
    // Prevent rapid ARM attempts
    if (currentTime - lastArmAttempt < ARM_RETRY_DELAY) {
        Serial.println("ARM button pressed too quickly, please wait...");
        return;
    }
    lastArmAttempt = currentTime;
    
    if (!controller_armed) {
        // Try to ARM
        int throttle_value = map(throttle_raw, THROTTLE_MIN_ADC, THROTTLE_MAX_ADC, THROTTLE_MIN, THROTTLE_MAX);
        throttle_value = constrain(throttle_value, THROTTLE_MIN, THROTTLE_MAX);
        
        Serial.printf("ARM ATTEMPT: Throttle=%d (ADC=%d)\n", throttle_value, throttle_raw);
        
        if (throttle_value < 30) {  // Safety: only arm at low throttle
            controller_armed = true;
            sendCommand(1, 0, current_roll, current_pitch, 0);  // ARM with 0 throttle and yaw
            Serial.println("*** ARM COMMAND SENT *** - Waiting for drone confirmation...");
            
            // Wait for drone confirmation (non-blocking check)
            unsigned long armStart = millis();
            int confirmAttempts = 0;
            while (millis() - armStart < 2000 && confirmAttempts < 5) {  // Wait up to 2 seconds, max 5 attempts
                delay(100);
                requestStatus();  // Request immediate status update
                delay(100);
                
                if (status_received && receivedStatus.armed) {
                    drone_confirmed_armed = true;
                    Serial.println("✓ DRONE ARM CONFIRMED - Ready to fly!");
                    updateArmLED();  // Update LED immediately
                    return;
                }
                confirmAttempts++;
            }
            
            // If we get here, ARM failed
            Serial.println("✗ ARM FAILED - No confirmation from drone");
            Serial.println("  Check: 1) Drone powered? 2) Drone calibrated? 3) Communication OK?");
            controller_armed = false;  // Reset controller state
            updateArmLED();  // Update LED to show failure
            
        } else {
            Serial.printf("✗ Cannot ARM - Throttle too high (%d)! Lower joystick to bottom first.\n", throttle_value);
        }
    } else {
        // DISARM
        controller_armed = false;
        drone_confirmed_armed = false;
        sendCommand(2, 0, 0, 0, 0);  // DISARM command
        current_throttle = 0;
        current_yaw = 0.0;  // Reset yaw as well
        Serial.println("*** DISARM COMMAND SENT ***");
        updateArmLED();  // Update LED immediately
        
        // Brief wait for confirmation
        delay(200);
        requestStatus();
        delay(200);
        
        if (status_received && !receivedStatus.armed) {
            Serial.println("✓ DRONE DISARM CONFIRMED");
        } else {
            Serial.println("⚠ DISARM sent but no confirmation (drone may still disarm)");
        }
    }
}

void checkArmSync() {
    if (!status_received) {
        return;  // No drone communication
    }
    
    // Debug: Always show what we think vs what drone reports
    static unsigned long lastSyncDebug = 0;
    if (millis() - lastSyncDebug > 5000) {  // Every 5 seconds
        Serial.printf("SYNC: Controller=%s, Drone=%s, LED=%s, Comm=%lums ago\n",
                      controller_armed ? "ARMED" : "DISARMED",
                      receivedStatus.armed ? "ARMED" : "DISARMED",
                      digitalRead(ARM_LED_PIN) ? "ON" : "OFF",
                      millis() - lastDroneResponse);
        lastSyncDebug = millis();
    }
    
    // Check if controller and drone are out of sync
    if (controller_armed != receivedStatus.armed) {
        Serial.printf("⚠ ARM SYNC ERROR: Controller=%s but Drone=%s\n",
                      controller_armed ? "ARMED" : "DISARMED",
                      receivedStatus.armed ? "ARMED" : "DISARMED");
        
        // Sync controller to drone state (drone is authoritative)
        controller_armed = receivedStatus.armed;
        drone_confirmed_armed = receivedStatus.armed;
        
        if (receivedStatus.armed) {
            Serial.println("→ Synced: Controller now ARMED to match drone");
        } else {
            Serial.println("→ Synced: Controller now DISARMED to match drone");
            current_throttle = 0;
            current_yaw = 0.0;  // Ensure both controls are zero when disarmed
        }
        updateArmLED();  // Update LED when syncing
    } else {
        // States match, update confirmed status
        drone_confirmed_armed = receivedStatus.armed;
    }
}

void updateCommunicationStatus() {
    if (status_received) {
        lastDroneResponse = millis();
        if (!communication_active) {
            communication_active = true;
            Serial.println("✓ Drone communication established");
        }
    } else if (communication_active && (millis() - lastDroneResponse > 3000)) {
        communication_active = false;
        Serial.println("✗ Drone communication lost!");
        
        // Safety: Disarm if communication is lost
        if (controller_armed) {
            controller_armed = false;
            drone_confirmed_armed = false;
            current_throttle = 0;
            current_yaw = 0.0;  // Reset both controls
            updateArmLED();  // Update LED when auto-disarming
            Serial.println("→ Auto-disarmed due to communication loss");
        }
    }
}

void handleSerialCommands() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        input.toLowerCase();
        
        if (input.startsWith("r ")) {
            float roll = input.substring(2).toFloat();
            current_roll = constrain(roll, -30.0, 30.0);
            Serial.printf("Roll set to %.1f degrees\n", current_roll);
        }
        else if (input.startsWith("p ")) {
            float pitch = input.substring(2).toFloat();
            current_pitch = constrain(pitch, -30.0, 30.0);
            Serial.printf("Pitch set to %.1f degrees\n", current_pitch);
        }
        else if (input == "center") {
            current_roll = 0.0;
            current_pitch = 0.0;
            Serial.println("Roll and pitch centered (yaw controlled by joystick)");
        }
        else if (input == "status") {
            printDetailedStatus();
        }
        else if (input == "sync") {
            Serial.println("Force syncing ARM state with drone...");
            requestStatus();
            delay(500);
            checkArmSync();
        }
        else if (input == "check") {
            performPreflightCheck();
        }
        else if (input == "cal") {
            calibrateJoystick();
        }
        else if (input == "help") {
            printHelp();
        }
        else if (input == "arm") {
            if (!controller_armed) {
                int throttle_raw = analogRead(THROTTLE_PIN);
                toggleArmState(throttle_raw);
            } else {
                Serial.println("Already armed. Press button or type 'disarm' to disarm.");
            }
        }
        else if (input == "disarm") {
            if (controller_armed) {
                controller_armed = false;
                drone_confirmed_armed = false;
                sendCommand(2, 0, 0, 0, 0);
                current_throttle = 0;
                current_yaw = 0.0;  // Reset yaw as well
                updateArmLED();  // Update LED immediately
                Serial.println("*** DISARMED *** (via serial command)");
            } else {
                Serial.println("Already disarmed.");
            }
        }
        else {
            Serial.println("Unknown command. Type 'help' for available commands.");
        }
    }
}

void sendCommand(uint8_t cmd, int16_t throttle, float roll, float pitch, float yaw) {
    sendData.command = cmd;
    sendData.throttle = throttle;
    sendData.roll_setpoint = roll;
    sendData.pitch_setpoint = pitch;
    sendData.yaw_setpoint = yaw;
    sendData.timestamp = millis();
    
    esp_err_t result = esp_now_send(droneMAC, (uint8_t*)&sendData, sizeof(sendData));
    if (result != ESP_OK) {
        Serial.printf("✗ ESP-NOW TX failed! Error: %d\n", result);
        status_received = false;
    }
}

void requestStatus() {
    // Send status request (command 0)
    sendCommand(0, current_throttle, current_roll, current_pitch, current_yaw);
}

void printDetailedStatus() {
    Serial.println("\n=== Controller Status ===");
    Serial.printf("Communication: %s\n", communication_active ? "CONNECTED" : "DISCONNECTED");
    
    if (status_received) {
        Serial.printf("Last drone response: %lu ms ago\n", millis() - lastDroneResponse);
        Serial.printf("Drone ARM state: %s\n", receivedStatus.armed ? "ARMED" : "DISARMED");
        Serial.printf("Drone throttle: %d (%d%%)\n", receivedStatus.throttle, (receivedStatus.throttle * 100) / 255);
        Serial.printf("Drone attitude: Roll=%.1f°, Pitch=%.1f°\n", receivedStatus.roll, receivedStatus.pitch);
    } else {
        Serial.println("No drone response received yet");
    }
    
    Serial.println("\n=== Controller Hardware ===");
    Serial.printf("Controller ARM state: %s\n", controller_armed ? "ARMED" : "DISARMED");
    Serial.printf("ARM states synced: %s\n", (controller_armed == drone_confirmed_armed) ? "YES" : "NO");
    Serial.printf("ARM Button (GP11): %s\n", digitalRead(ARM_BUTTON_PIN) ? "Not Pressed" : "PRESSED");
    Serial.printf("ARM LED (GP48): %s\n", digitalRead(ARM_LED_PIN) ? "ON" : "OFF");
    Serial.printf("Throttle Joystick (GP13): %d (raw ADC)\n", analogRead(THROTTLE_PIN));
    Serial.printf("Yaw Joystick (GP12): %d (raw ADC)\n", analogRead(YAW_PIN));
    Serial.printf("Current commands: T=%d (%d%%), Y=%.1f°/s\n", current_throttle, (current_throttle * 100) / 255, current_yaw);
    Serial.printf("Attitude commands: R=%.1f°, P=%.1f°\n", current_roll, current_pitch);
    Serial.println("=====================\n");
}

void printHelp() {
    Serial.println("\n=== Hardware Controls ===");
    Serial.println("VRY Joystick (GP13)  - Throttle: 0-255 PWM (when armed)");
    Serial.println("VRX Joystick (GP12)  - Yaw rate: -100 to +100 deg/s (when armed)");
    Serial.println("Pitch Control (GP9)  - Pitch angle: -30 to +30 degrees (when armed)");
    Serial.println("Roll Control (GP8)   - Roll angle: -30 to +30 degrees (when armed)");
    Serial.println("Toggle Button (GP11) - Press to ARM/DISARM drone");
    Serial.println("ARM LED (GP48)       - Shows ARM status (ON=ARMED, OFF=DISARMED)");
    Serial.println();
    Serial.println("=== Serial Commands ===");
    Serial.println("r <-30 to 30>        - Set roll angle (degrees)");
    Serial.println("p <-30 to 30>        - Set pitch angle (degrees)");
    Serial.println("center               - Level roll/pitch (yaw via joystick)");
    Serial.println("arm                  - ARM drone (if throttle low)");
    Serial.println("disarm               - DISARM drone");
    Serial.println("status               - Show detailed status");
    Serial.println("sync                 - Force ARM state sync");
    Serial.println("check                - Preflight check");
    Serial.println("cal                  - Calibrate joystick range");
    Serial.println("help                 - Show this help");
    Serial.println("====================\n");
}

void performPreflightCheck() {
    Serial.println("\n=== Preflight Check ===");
    
    // Check communication
    Serial.println("1. Testing communication...");
    requestStatus();
    delay(1000);  // Wait longer for response
    
    if (status_received && communication_active) {
        Serial.println("✓ Drone communication OK");
        Serial.printf("✓ Signal strength: Good (last response %lu ms ago)\n", millis() - lastDroneResponse);
    } else {
        Serial.println("✗ No response from drone");
        Serial.println("  Check: Power, MAC address, distance");
    }
    
    // Check hardware
    Serial.println("2. Checking hardware...");
    Serial.printf("✓ ARM Button (GP11): %s\n", digitalRead(ARM_BUTTON_PIN) ? "Not pressed" : "PRESSED");
    Serial.printf("✓ ARM LED (GP48): %s\n", digitalRead(ARM_LED_PIN) ? "ON" : "OFF");
    
    int throttle_reading = analogRead(THROTTLE_PIN);
    int throttle_mapped = map(throttle_reading, THROTTLE_MIN_ADC, THROTTLE_MAX_ADC, THROTTLE_MIN, THROTTLE_MAX);
    Serial.printf("✓ Throttle Joystick (GP13): %d → %d PWM\n", throttle_reading, throttle_mapped);
    
    int yaw_reading = analogRead(YAW_PIN);
    float yaw_mapped = 0.0;
    if (abs(yaw_reading - YAW_CENTER_ADC) > YAW_DEADZONE) {
        yaw_mapped = ((float)(yaw_reading - YAW_CENTER_ADC) / (YAW_MAX_ADC / 2.0)) * YAW_RATE_MAX;
        yaw_mapped = constrain(yaw_mapped, -YAW_RATE_MAX, YAW_RATE_MAX);
    }
    Serial.printf("✓ Yaw Joystick (GP12): %d → %.1f°/s\n", yaw_reading, yaw_mapped);
    
    Serial.println("======================\n");
}

void initESPNow() {
    // Set WiFi mode and channel
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    
    // Display MAC addresses
    Serial.print("Controller MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.println("Make sure droneMAC[] matches your drone's MAC address!");
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed!");
        return;
    }
    
    // Register callbacks
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataReceived);
    
    // Add drone as peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, droneMAC, 6);
    peerInfo.channel = 1;
    peerInfo.ifidx = WIFI_IF_STA;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add drone as ESP-NOW peer!");
        return;
    }
    
    Serial.println("ESP-NOW initialized successfully");
}

// ESP-NOW callback when data is sent (updated for newer ESP32 core)
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    if (status != ESP_NOW_SEND_SUCCESS) {
        Serial.println("ESP-NOW send failed!");
        status_received = false;
    }
}

// ESP-NOW callback when data is received (updated for newer ESP32 core)
void onDataReceived(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
    if (len == sizeof(esp_now_response_t)) {
        memcpy(&receivedStatus, incomingData, sizeof(receivedStatus));
        status_received = true;
        
        // Debug: Print received status for debugging
        Serial.printf("RX: Armed=%s, T=%d, Roll=%.1f, Pitch=%.1f\n", 
                     receivedStatus.armed ? "YES" : "NO", receivedStatus.throttle,
                     receivedStatus.roll, receivedStatus.pitch);
    } else {
        Serial.printf("RX ERROR: Expected %d bytes, got %d\n", sizeof(esp_now_response_t), len);
    }
}