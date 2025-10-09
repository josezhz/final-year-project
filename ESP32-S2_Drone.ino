#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

MPU6050 mpu;

// ESP32-S2-Drone V1.2 specific pin definitions - UPDATED MOTOR MAPPING
#define MOTOR_FR_PIN     5    // Front Right Motor (M1)  
#define MOTOR_BR_PIN     6    // Back Right Motor (M2)
#define MOTOR_BL_PIN     3    // Back Left Motor (M3)
#define MOTOR_FL_PIN     4    // Front Left Motor (M4)

// IMU pins for ESP32-S2-Drone V1.2
#define IMU_SDA_PIN      11   // I2C Data
#define IMU_SCL_PIN      10   // I2C Clock
#define IMU_INT_PIN      12   // MPU6050 Interrupt

// LED indicators
#define LED_RED_PIN      8    // Red status LED
#define LED_GREEN_PIN    9    // Green status LED
#define LED_BLUE_PIN     7    // Blue status LED

// BUZZER pins for audio feedback
#define BUZZER_POS_PIN   39   // BUZ_1 (BUZZ+)
#define BUZZER_NEG_PIN   38   // BUZ_2 (BUZZ-)

// BATTERY monitoring pin  
#define BATTERY_PIN      2    // ADC_7_BAT (VBAT/2)

// EXTERNAL I/O pin for additional features
#define EXT_IO_PIN       1    // EXT_IO1

// Battery thresholds for 2S LiPo (7.4V nominal)
#define BATTERY_CRITICAL 6.0f  // Auto-disarm level
#define BATTERY_LOW      6.6f  // Warning level
#define BATTERY_GOOD     7.0f  // Good level

// PWM Configuration for ESP32 Arduino Core 3.x
#define PWM_FREQ         20000
#define PWM_RESOLUTION   8     // 8-bit resolution (0-255)

// ESP-NOW Communication Structure (must match controller exactly)
typedef struct {
    uint8_t command;        // 1=ARM, 2=DISARM, 3=THROTTLE, 4=ATTITUDE, 5=EMERGENCY
    int16_t throttle;       // 0-255 PWM value
    float roll_setpoint;    // Roll angle setpoint (-30 to +30 degrees)
    float pitch_setpoint;   // Pitch angle setpoint (-30 to +30 degrees)
    float yaw_setpoint;     // Yaw rate setpoint (-100 to +100 deg/s)
    uint32_t timestamp;     // For timeout detection
} esp_now_data_t;

// ESP-NOW Response Structure (drone status back to controller)
typedef struct {
    bool armed;
    int16_t throttle;
    float roll;
    float pitch;
    float battery_voltage;  // Future use
    uint8_t signal_strength;
    uint32_t timestamp;
} esp_now_response_t;

// Kalman filter class
class KalmanFilter {
private:
    float Q_angle, Q_bias, R_measure;
    float angle, bias, rate;
    float P[2][2];
    
public:
    KalmanFilter() {
        Q_angle = 0.5f;
        Q_bias = 0.1f;
        R_measure = 0.005f;
        angle = 0.0f;
        bias = 0.0f;
        P[0][0] = 0.0f;
        P[0][1] = 0.0f;
        P[1][0] = 0.0f;
        P[1][1] = 0.0f;
    }
    
    float getAngle(float newAngle, float newRate, float dt) {
        rate = newRate - bias;
        angle += dt * rate;
        
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
        P[0][1] -= dt * P[1][1];
        P[1][0] -= dt * P[1][1];
        P[1][1] += Q_bias * dt;
        
        float S = P[0][0] + R_measure;
        float K[2];
        K[0] = P[0][0] / S;
        K[1] = P[1][0] / S;
        
        float y = newAngle - angle;
        angle += K[0] * y;
        bias += K[1] * y;
        
        float P00_temp = P[0][0];
        float P01_temp = P[0][1];
        
        P[0][0] -= K[0] * P00_temp;
        P[0][1] -= K[0] * P01_temp;
        P[1][0] -= K[1] * P00_temp;
        P[1][1] -= K[1] * P01_temp;
        
        return angle;
    }
    
    void setAngle(float angle) { this->angle = angle; }
    void reset() { 
        angle = 0.0f; 
        bias = 0.0f; 
        memset(P, 0, sizeof(P));
    }
};

// PID Controller class
class PIDController {
private:
    float kp, ki, kd;
    float integral, previousError, previousInput;  // Added previousInput
    float integralLimit;
    float derivativeFilter;                        // Added derivativeFilter
    float filteredDerivative;                      // Added filteredDerivative
    
public:
    PIDController(float p, float i, float d, float intLimit = 50.0, float derivFilter = 0.1) {
        kp = p; 
        ki = i; 
        kd = d;
        integralLimit = intLimit;
        derivativeFilter = derivFilter;
        reset();
    }
    
    float compute(float setpoint, float input, float dt) {
        if (dt <= 0) return 0;
        
        float error = setpoint - input;
        
        // Proportional term
        float proportional = kp * error;
        
        // Integral term with windup protection
        integral += error * dt;
        integral = constrain(integral, -integralLimit, integralLimit);
        float integralTerm = ki * integral;
        
        // Derivative term - using input derivative to avoid derivative kick
        float inputDerivative = -(input - previousInput) / dt;
        
        // Apply low-pass filter to derivative for noise reduction
        filteredDerivative = (derivativeFilter * filteredDerivative) + ((1.0 - derivativeFilter) * inputDerivative);
        float derivativeTerm = kd * filteredDerivative;
        
        // Store values for next iteration
        previousError = error;
        previousInput = input;
        
        return proportional + integralTerm + derivativeTerm;
    }
    
    void reset() {
        integral = 0.0f;
        previousError = 0.0f;
        previousInput = 0.0f;      // Now properly declared
        filteredDerivative = 0.0f; // Now properly declared
    }
    
    void setTunings(float p, float i, float d) {
        kp = p; 
        ki = i; 
        kd = d;
    }
    
    void setLimits(float intLimit) {
        integralLimit = intLimit;
    }
    
    void setDerivativeFilter(float filter) {
        derivativeFilter = constrain(filter, 0.0, 0.99);  // Now properly declared
    }
    
    // Getter functions for debugging
    float getProportional(float error) { return kp * error; }
    float getIntegral() { return ki * integral; }
    float getDerivative() { return kd * filteredDerivative; }  // Now properly declared
    float getIntegralSum() { return integral; }
};

// Flight control objects
KalmanFilter kalmanX, kalmanY, kalmanZ;
PIDController pidRoll(0.15, 0.05, 0, 50, 0.0);
PIDController pidPitch(0.15, 0.05, 0, 50, 0.0);
PIDController pidYaw(0, 0, 0, 50, 0.0);

// Flight state variables
float roll = 0, pitch = 0, yaw = 0;
float rollRate = 0, pitchRate = 0, yawRate = 0;
float rollSetpoint = 0, pitchSetpoint = 0, yawSetpoint = 0;

// Motor control - PWM values (0-255)
int throttle = 0;
int motorFL_pwm, motorFR_pwm, motorBL_pwm, motorBR_pwm;

// Motor calibration multipliers (adjust for individual motor differences)
float motorFL_multiplier = 1.0f;  // Front Left motor calibration
float motorFR_multiplier = 1.0f;  // Front Right motor calibration 
float motorBL_multiplier = 1.0f;  // Back Left motor calibration
float motorBR_multiplier = 1.0f;  // Back Right motor calibration

// Timing and performance
unsigned long loopTimer = 0;
float dt = 0;

// ESP-NOW variables
esp_now_data_t receivedData;
esp_now_response_t responseData;
bool esp_now_data_received = false;
unsigned long lastESPNowReceived = 0;
unsigned long lastStatusSent = 0;
const unsigned long ESP_NOW_TIMEOUT = 1000;  // 1 second timeout

// Flight modes and safety
bool armed = false;
bool calibrated = false;

// Constants for PWM control
const int MIN_THROTTLE = 0;      // 0% PWM - motors off
const int MAX_THROTTLE = 255;    // 100% PWM - full power
const int MOTOR_OFF = 0;         // 0% PWM - completely off
const int MOTOR_IDLE = 10;       // ~5% PWM - minimum spin when armed
const int CONTROL_FREQUENCY = 500;
const unsigned long CONTROL_PERIOD = 1000000 / CONTROL_FREQUENCY;

// Controller MAC address - UPDATE THIS WITH YOUR ESP32-S3 CONTROLLER MAC!
uint8_t controllerMAC[] = {0x50, 0x78, 0x7d, 0x19, 0x9d, 0xf0};

// Communication tracking
bool communication_established = false;

float batteryVoltage = 0.0f;
unsigned long lastBatteryCheck = 0;
unsigned long lastBatteryWarning = 0;
bool batteryLowWarningActive = false;

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32-S2-Drone V1.2 Flight Controller Starting...");
    Serial.println("Motor Type: Brushed DC with MOSFET PWM Control");
    Serial.println("Communication: ESP-NOW (Fixed Channel Sync)");
    Serial.println("Coordinate System: Front = -Y axis, Roll FLIPPED");
    
    // Configure GPIO pins
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT); 
    pinMode(LED_BLUE_PIN, OUTPUT);
    
    setStatusLED(255, 0, 0);  // Red during initialization
    
    // Initialize ESP-NOW FIRST (before other WiFi operations)
    initESPNow();
    
    // Initialize I2C
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(400000);
    
    // Initialize MPU6050
    Serial.println("Initializing MPU6050...");
    mpu.initialize();
    
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        setStatusLED(255, 0, 0);
        while(1) {
            delay(1000);
            Serial.println("Check IMU connection!");
        }
    }
    
    Serial.println("MPU6050 initialized successfully!");
    
    // Initialize PWM for brushed DC motors
    Serial.println("Initializing PWM for brushed DC motors...");
    Serial.println("Motor Mapping: FL=GPIO4, FR=GPIO5, BR=GPIO6, BL=GPIO3");
    Serial.println("Motor Calibration Multipliers: All set to 1.0 (adjustable)");
    setupPWMChannels();
    
    // Start with motors OFF
    setAllMotorsOff();
    delay(1000);
    setStatusLED(255, 255, 0);
    
    // Calibrate sensors
    Serial.println("Calibrating sensors... Keep drone level!");
    calibrateSensors();
    calibrated = true;
    
    // Ready
    setAllMotorsOff();
    setStatusLED(0, 255, 0);  // Green = Ready
    Serial.println("ESP32-S2-Drone V1.2 Ready!");
    Serial.println("Commands: ARM, DISARM, T+, T-, CAL, TEST, STOP");
    Serial.println("Motor Cal: CAL_FL_1.05, CAL_FR_0.95, CAL_BL_1.02, CAL_BR_0.98");
    Serial.println("ESP-NOW Controller Communication Active!");
    Serial.println("BRUSHED DC MOTORS ARE OFF - SAFE TO HANDLE");
    
    loopTimer = micros();
}

void setupPWMChannels() {
    // Arduino Core 3.x syntax - using ledcAttach()
    if (!ledcAttach(MOTOR_FL_PIN, PWM_FREQ, PWM_RESOLUTION)) {
        Serial.println("Failed to attach FL motor PWM (GPIO4)");
    }
    if (!ledcAttach(MOTOR_FR_PIN, PWM_FREQ, PWM_RESOLUTION)) {
        Serial.println("Failed to attach FR motor PWM (GPIO5)");
    }
    if (!ledcAttach(MOTOR_BL_PIN, PWM_FREQ, PWM_RESOLUTION)) {
        Serial.println("Failed to attach BL motor PWM (GPIO3)");
    }
    if (!ledcAttach(MOTOR_BR_PIN, PWM_FREQ, PWM_RESOLUTION)) {
        Serial.println("Failed to attach BR motor PWM (GPIO6)");
    }
    
    Serial.printf("PWM Setup: %dkHz, %d-bit resolution\n", PWM_FREQ/1000, PWM_RESOLUTION);
}

void loop() {
    if (micros() - loopTimer < CONTROL_PERIOD) return;
    
    dt = (float)(micros() - loopTimer) / 1000000.0f;
    loopTimer = micros();
    
    if (calibrated) {
        readIMUData();
        
        if (armed) {
            performFlightControl();
            setStatusLED(0, 0, 255);  // Blue when flying
        } else {
            setAllMotorsOff();
            resetPIDs();
            setStatusLED(communication_established ? 0 : 255, 255, 0);  // Green when disarmed & connected, yellow if no connection
        }
    } else {
        setAllMotorsOff();
    }
    
    handleSerialCommands();
    handleESPNowCommands();
    
    // Print status less frequently when armed to reduce spam
    static unsigned long lastStatus = 0;
    unsigned long statusInterval = armed ? 1000 : 1000;
    if (millis() - lastStatus > statusInterval) {
        //printFlightStatus();
        lastStatus = millis();
    }

    // Check battery every 5 seconds
    if (millis() - lastBatteryCheck > 5000) {
        //checkBatteryLevel();
        updateEnhancedStatus();
        lastBatteryCheck = millis();
    }
}

void readIMUData() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    float accX = (float)ax / 16384.0f;
    float accY = (float)ay / 16384.0f;
    float accZ = (float)az / 16384.0f;
    
    // Updated coordinate system: Front = -Y axis, Roll direction FLIPPED
    // Roll about -Y axis (FLIPPED - now positive), Pitch about X axis
    rollRate = (float)gy / 131.0f;    // Roll about -Y axis (FLIPPED - removed negation)
    pitchRate = (float)gx / 131.0f;   // Pitch about X axis
    yawRate = (float)gz / 131.0f;     // Yaw about Z axis (unchanged)
    
    // Calculate angles with updated coordinate system (FLIPPED roll)
    // Roll: rotation about -Y axis (FLIPPED), Pitch: rotation about X axis
    float roll_acc = atan2(accX, accZ) * RAD_TO_DEG;   // Roll FLIPPED (removed negation)
    float pitch_acc = atan2(accY, accZ) * RAD_TO_DEG;  // Pitch about X
    
    roll = kalmanX.getAngle(roll_acc, rollRate, dt);
    pitch = kalmanY.getAngle(pitch_acc, pitchRate, dt);
}

void performFlightControl() {
    if (!armed) {
        setAllMotorsOff();
        return;
    }
    
    float rollCorrection = pidRoll.compute(rollSetpoint, roll, dt);
    float pitchCorrection = pidPitch.compute(pitchSetpoint, pitch, dt);
    float yawCorrection = pidYaw.compute(yawSetpoint, yawRate, dt);

    int motorFL_correction = + rollCorrection - pitchCorrection - yawCorrection;
    int motorFR_correction = - rollCorrection - pitchCorrection + yawCorrection;
    int motorBL_correction = + rollCorrection + pitchCorrection + yawCorrection;
    int motorBR_correction = - rollCorrection + pitchCorrection - yawCorrection;

    Serial.print("Correction: FL:");
        Serial.print(motorFL_correction);
        Serial.print(" FR:"); 
        Serial.print(motorFR_correction);
        Serial.print(" BL:");
        Serial.print(motorBL_correction);
        Serial.print(" BR:");
        Serial.print(motorBR_correction);
        Serial.println();

    // Constrain from raw motor range to PWM range (MOTOR_IDLE to MAX_THROTTLE)
    motorFL_pwm = constrain((throttle + map(motorFL_correction, -100, 100, -200, 200)) * motorFL_multiplier, MOTOR_IDLE, MAX_THROTTLE);
    motorFR_pwm = constrain((throttle + map(motorFR_correction, -100, 100, -200, 200)) * motorFR_multiplier, MOTOR_IDLE, MAX_THROTTLE);
    motorBL_pwm = constrain((throttle + map(motorBL_correction, -100, 100, -200, 200)) * motorBL_multiplier, MOTOR_IDLE, MAX_THROTTLE);
    motorBR_pwm = constrain((throttle + map(motorBR_correction, -100, 100, -200, 200)) * motorBR_multiplier, MOTOR_IDLE, MAX_THROTTLE);
    
    // Arduino Core 3.x - write PWM using pin numbers
    ledcWrite(MOTOR_FL_PIN, motorFL_pwm);
    ledcWrite(MOTOR_FR_PIN, motorFR_pwm);
    ledcWrite(MOTOR_BL_PIN, motorBL_pwm);
    ledcWrite(MOTOR_BR_PIN, motorBR_pwm);
}

void calibrateSensors() {
    setAllMotorsOff();
    
    const int samples = 2000;
    long gyroX_offset = 0, gyroY_offset = 0, gyroZ_offset = 0;
    
    setStatusLED(255, 165, 0);
    
    for (int i = 0; i < samples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        
        gyroX_offset += gx;
        gyroY_offset += gy; 
        gyroZ_offset += gz;
        
        if (i % 200 == 0) {
            Serial.print("Calibrating... "); 
            Serial.print((i * 100) / samples);
            Serial.println("%");
        }
        delay(2);
    }
    
    Serial.printf("Gyro offsets: X=%ld Y=%ld Z=%ld\n", 
                  gyroX_offset / samples, gyroY_offset / samples, gyroZ_offset / samples);
    
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    float accX = (float)ax / 16384.0f;
    float accY = (float)ay / 16384.0f; 
    float accZ = (float)az / 16384.0f;
    
    // Updated for new coordinate system (FLIPPED roll)
    roll = atan2(accX, accZ) * RAD_TO_DEG;    // Roll FLIPPED (removed negation)
    pitch = atan2(accY, accZ) * RAD_TO_DEG;   // Pitch about X axis
    
    kalmanX.setAngle(roll);
    kalmanY.setAngle(pitch);
    
    Serial.println("Sensor calibration complete!");
    Serial.println("Coordinate System: Front = -Y axis, Roll FLIPPED, Pitch about X");
}

void initESPNow() {
    // Set device as WiFi Station and configure channel
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    // CRITICAL: Set same channel as controller (channel 1)
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    
    // Print MAC address for reference
    Serial.print("Drone MAC Address: ");
    Serial.println(WiFi.macAddress());
    Serial.printf("Update controllerMAC[] with your ESP32-S3 controller MAC!\n");
    Serial.printf("Current controller MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  controllerMAC[0], controllerMAC[1], controllerMAC[2], 
                  controllerMAC[3], controllerMAC[4], controllerMAC[5]);
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    
    // Register callback functions
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataReceived);
    
    // Add controller as peer with matching settings
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, controllerMAC, 6);
    peerInfo.channel = 1;           // MATCH CONTROLLER CHANNEL
    peerInfo.ifidx = WIFI_IF_STA;   // MATCH CONTROLLER INTERFACE
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add ESP-NOW peer");
        return;
    }
    
    Serial.println("ESP-NOW initialized successfully on channel 1");
}

// Callback when data is sent (with detailed debugging)
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        // Status sent successfully
        lastStatusSent = millis();
    } else {
        Serial.printf("ESP-NOW send failed! Status: %d\n", status);
    }
}

// Callback when data is received (with detailed debugging)
void onDataReceived(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
    if (len == sizeof(esp_now_data_t)) {
        memcpy(&receivedData, incomingData, sizeof(receivedData));
        esp_now_data_received = true;
        lastESPNowReceived = millis();
        
        if (!communication_established) {
            communication_established = true;
            Serial.println("✓ ESP-NOW communication established with controller!");
        }
        
        // DEBUG: Log what we received
        const char* cmdNames[] = {"STATUS_REQ", "ARM", "DISARM", "THROTTLE", "ATTITUDE", "EMERGENCY"};
        Serial.printf("RX: %s T=%d R=%.1f P=%.1f Y=%.1f\n", 
                      (receivedData.command <= 5) ? cmdNames[receivedData.command] : "UNKNOWN",
                      receivedData.throttle, receivedData.roll_setpoint, 
                      receivedData.pitch_setpoint, receivedData.yaw_setpoint);
        
        // Always send response back to controller (critical for sync)
        sendStatusResponse();
    } else {
        Serial.printf("RX ERROR: Expected %d bytes, got %d\n", sizeof(esp_now_data_t), len);
    }
}

void sendStatusResponse() {
    responseData.armed = armed;
    responseData.throttle = throttle;
    responseData.roll = roll;
    responseData.pitch = pitch;
    responseData.battery_voltage = 3.7; // Placeholder
    responseData.signal_strength = 100; // Placeholder
    responseData.timestamp = millis();
    
    // DEBUG: Log what we're sending back
    Serial.printf("TX: Armed=%s T=%d R=%.1f P=%.1f\n",
                  armed ? "YES" : "NO", throttle, roll, pitch);
    
    esp_err_t result = esp_now_send(controllerMAC, (uint8_t*)&responseData, sizeof(responseData));
    if (result != ESP_OK) {
        Serial.printf("Status send failed! Error: %d\n", result);
        communication_established = false;
    }
}

void handleESPNowCommands() {
    // Check for ESP-NOW timeout
    if (armed && communication_established && (millis() - lastESPNowReceived > ESP_NOW_TIMEOUT)) {
        Serial.println("ESP-NOW connection lost! Emergency landing...");
        armed = false;
        throttle = MIN_THROTTLE;
        setAllMotorsOff();
        setStatusLED(255, 0, 0); // Red = Error
        communication_established = false;
        return;
    }
    
    if (!esp_now_data_received) return;
    esp_now_data_received = false;
    
    // Process received commands
    switch (receivedData.command) {
        case 0: // STATUS REQUEST (don't change anything, just send response)
            // Response already sent in onDataReceived
            break;
            
        case 1: // ARM
            Serial.printf("ARM request: throttle=%d, calibrated=%s\n", 
                          receivedData.throttle, calibrated ? "YES" : "NO");
            if (receivedData.throttle <= MOTOR_IDLE && calibrated) {
                armed = true;
                resetPIDs();
                playArmingSequence();  // Add this line
                Serial.println("✓ ESP-NOW: ARMED");
            } else {
                Serial.printf("✗ ARM FAILED: throttle=%d (must be ≤%d), calibrated=%s\n", 
                              receivedData.throttle, MOTOR_IDLE, calibrated ? "YES" : "NO");
            }
            break;
            
        case 2: // DISARM
            armed = false;
            throttle = MIN_THROTTLE;
            rollSetpoint = pitchSetpoint = yawSetpoint = 0;
            setAllMotorsOff();
            playDisarmSequence();  // Add this line
            Serial.println("✓ ESP-NOW: DISARMED");
            break;
            
        case 3: // THROTTLE UPDATE (legacy)
            throttle = constrain(receivedData.throttle, MIN_THROTTLE, MAX_THROTTLE);
            Serial.printf("ESP-NOW: Throttle = %d\n", throttle);
            break;
            
        case 4: // ATTITUDE CONTROL (main flight command)
            throttle = constrain(receivedData.throttle, MIN_THROTTLE, MAX_THROTTLE);
            rollSetpoint = constrain(receivedData.roll_setpoint, -30.0, 30.0);
            pitchSetpoint = constrain(receivedData.pitch_setpoint, -30.0, 30.0);
            yawSetpoint = constrain(receivedData.yaw_setpoint, -100.0, 100.0);
            // Reduced logging when armed to prevent spam
            if (!armed || (millis() % 2000 < 100)) {  // Log every 2 seconds when armed
                Serial.printf("Flight: T=%d R=%.1f P=%.1f Y=%.1f\n", 
                             throttle, rollSetpoint, pitchSetpoint, yawSetpoint);
            }
            break;
            
        case 5: // EMERGENCY STOP
            armed = false;
            throttle = MIN_THROTTLE;
            rollSetpoint = pitchSetpoint = yawSetpoint = 0;
            setAllMotorsOff();
            Serial.println("*** ESP-NOW: EMERGENCY STOP ***");
            break;
            
        default:
            Serial.printf("Unknown command: %d\n", receivedData.command);
            break;
    }
}

void handleSerialCommands() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toUpperCase();
        
        if (cmd == "ARM") {
            if (throttle <= MOTOR_IDLE && calibrated) {
                armed = true;
                resetPIDs();
                Serial.println("ARMED - Ready to fly! Motors will spin at idle PWM.");
            } else {
                Serial.println("Cannot arm: Lower throttle and ensure calibrated");
            }
        }
        else if (cmd == "DISARM") {
            armed = false;
            setAllMotorsOff();
            Serial.println("DISARMED - Brushed motors OFF (0% PWM)");
        }
        else if (cmd == "T+" && armed) {
            throttle = min(throttle + 10, MAX_THROTTLE);
            Serial.print("Throttle PWM: "); Serial.print(throttle); 
            Serial.print(" ("); Serial.print((throttle*100)/255); Serial.println("%)");
        }
        else if (cmd == "T-") {
            throttle = max(throttle - 10, MIN_THROTTLE);
            Serial.print("Throttle PWM: "); Serial.print(throttle);
            Serial.print(" ("); Serial.print((throttle*100)/255); Serial.println("%)");
        }
        else if (cmd == "CAL") {
            armed = false;
            setAllMotorsOff();
            calibrated = false;
            Serial.println("Recalibrating...");
            calibrateSensors();
            calibrated = true;
        }
        else if (cmd == "STATUS") {
            printDetailedStatus();
        }
        else if (cmd == "STOP" || cmd == "EMERGENCY") {
            armed = false;
            throttle = MIN_THROTTLE;
            setAllMotorsOff();
            Serial.println("EMERGENCY STOP - Brushed motors OFF");
        }
        else if (cmd == "TEST" && !armed) {
            testMotors();
        }
        else if (cmd == "COMM") {
            Serial.printf("Communication Status: %s\n", communication_established ? "ACTIVE" : "NO CONTROLLER");
            Serial.printf("Last received: %lums ago\n", millis() - lastESPNowReceived);
            Serial.printf("Last sent: %lums ago\n", millis() - lastStatusSent);
        }
        else if (cmd.startsWith("CAL_")) {
            // Motor calibration commands: CAL_FL_1.05, CAL_FR_0.95, etc.
            if (cmd.startsWith("CAL_FL_")) {
                float value = cmd.substring(7).toFloat();
                if (value >= 0.5 && value <= 1.5) {
                    motorFL_multiplier = value;
                    Serial.printf("FL motor multiplier set to %.3f\n", motorFL_multiplier);
                }
            }
            else if (cmd.startsWith("CAL_FR_")) {
                float value = cmd.substring(7).toFloat();
                if (value >= 0.5 && value <= 1.5) {
                    motorFR_multiplier = value;
                    Serial.printf("FR motor multiplier set to %.3f\n", motorFR_multiplier);
                }
            }
            else if (cmd.startsWith("CAL_BL_")) {
                float value = cmd.substring(7).toFloat();
                if (value >= 0.5 && value <= 1.5) {
                    motorBL_multiplier = value;
                    Serial.printf("BL motor multiplier set to %.3f\n", motorBL_multiplier);
                }
            }
            else if (cmd.startsWith("CAL_BR_")) {
                float value = cmd.substring(7).toFloat();
                if (value >= 0.5 && value <= 1.5) {
                    motorBR_multiplier = value;
                    Serial.printf("BR motor multiplier set to %.3f\n", motorBR_multiplier);
                }
            }
            else {
                Serial.println("Motor calibration usage:");
                Serial.println("CAL_FL_1.05  - Set FL motor to 105%");
                Serial.println("CAL_FR_0.95  - Set FR motor to 95%");
                Serial.println("CAL_BL_1.02  - Set BL motor to 102%");
                Serial.println("CAL_BR_0.98  - Set BR motor to 98%");
                Serial.printf("Current: FL=%.3f FR=%.3f BL=%.3f BR=%.3f\n",
                             motorFL_multiplier, motorFR_multiplier, 
                             motorBL_multiplier, motorBR_multiplier);
            }
        }
        else if (cmd == "BEEP") {
            playWarningBeep();
            Serial.println("Test beep played!");
        }
        else if (cmd == "BATTERY" || cmd == "BAT") {
            batteryVoltage = getBatteryVoltage();
            Serial.printf("Battery: %.2fV (%d%%) - %s\n", 
                        batteryVoltage, getBatteryPercentage(), getBatteryStatus().c_str());
        }
        else if (cmd == "STARTUP") {
            playStartupSequence();
            Serial.println("Startup sequence played");
        }
        else if (cmd == "LED_ON") {
            setExternalLED(true);
            Serial.println("External LED ON");
        }
        else if (cmd == "LED_OFF") {
            setExternalLED(false);
            Serial.println("External LED OFF");
        }
        else if (cmd == "SOUNDS") {
            Serial.println("Playing all sound sequences...");
            playStartupSequence();
            delay(500);
            playArmingSequence();
            delay(500);
            playDisarmSequence();
            delay(500);
            playBatteryWarning();
            delay(500);
            playWarningBeep();
        }
    }
}

void testMotors() {
    Serial.println("Testing brushed DC motors (DISARMED TEST)...");
    Serial.println("Each motor will spin briefly at low speed");
    Serial.println("Updated Motor Mapping: FL=GPIO4, FR=GPIO5, BR=GPIO6, BL=GPIO3");
    
    const int testPWM = 80;
    const int testDuration = 1000;
    
    Serial.println("Testing Front Left (GPIO4)...");
    ledcWrite(MOTOR_FL_PIN, testPWM);
    delay(testDuration);
    ledcWrite(MOTOR_FL_PIN, 0);
    delay(500);
    
    Serial.println("Testing Front Right (GPIO5)...");
    ledcWrite(MOTOR_FR_PIN, testPWM);
    delay(testDuration);
    ledcWrite(MOTOR_FR_PIN, 0);
    delay(500);
    
    Serial.println("Testing Back Left (GPIO3)...");
    ledcWrite(MOTOR_BL_PIN, testPWM);
    delay(testDuration);
    ledcWrite(MOTOR_BL_PIN, 0);
    delay(500);
    
    Serial.println("Testing Back Right (GPIO6)...");
    ledcWrite(MOTOR_BR_PIN, testPWM);
    delay(testDuration);
    ledcWrite(MOTOR_BR_PIN, 0);
    delay(500);
    
    Serial.println("Motor test complete!");
}

void setAllMotorsOff() {
    // Arduino Core 3.x - write 0% PWM using pin numbers
    ledcWrite(MOTOR_FL_PIN, MOTOR_OFF);
    ledcWrite(MOTOR_FR_PIN, MOTOR_OFF);
    ledcWrite(MOTOR_BL_PIN, MOTOR_OFF);
    ledcWrite(MOTOR_BR_PIN, MOTOR_OFF);
    
    motorFL_pwm = MOTOR_OFF;
    motorFR_pwm = MOTOR_OFF;
    motorBL_pwm = MOTOR_OFF;
    motorBR_pwm = MOTOR_OFF;
}

void setAllMotorsPWM(int pwmValue) {
    if (!armed) {
        setAllMotorsOff();
    } else {
        pwmValue = constrain(pwmValue, MOTOR_IDLE, MAX_THROTTLE);
        // Apply motor calibration multipliers
        ledcWrite(MOTOR_FL_PIN, (int)(pwmValue * motorFL_multiplier));
        ledcWrite(MOTOR_FR_PIN, (int)(pwmValue * motorFR_multiplier));
        ledcWrite(MOTOR_BL_PIN, (int)(pwmValue * motorBL_multiplier));
        ledcWrite(MOTOR_BR_PIN, (int)(pwmValue * motorBR_multiplier));
    }
}

void resetPIDs() {
    pidRoll.reset();
    pidPitch.reset();
    pidYaw.reset();
}

void setStatusLED(int r, int g, int b) {
    analogWrite(LED_RED_PIN, r);
    analogWrite(LED_GREEN_PIN, g);
    analogWrite(LED_BLUE_PIN, b);
}

void printFlightStatus() {
    Serial.printf("ARM:%s THR:%d(%d%%) R:%.1f P:%.1f FL:%d FR:%d BL:%d BR:%d COMM:%s\n",
                  armed ? "Y" : "N", 
                  throttle, (throttle*100)/255,
                  roll, pitch, 
                  motorFL_pwm, motorFR_pwm, motorBL_pwm, motorBR_pwm,
                  communication_established ? "OK" : "LOST");
}

void printDetailedStatus() {
    Serial.println("=== ESP32-S2-Drone V1.2 Enhanced Status ===");
    Serial.println("Motor Type: Brushed DC with MOSFET PWM Control");
    Serial.println("Communication: ESP-NOW (Channel 1 Sync Fixed)");
    Serial.println("Enhanced Features: Buzzer + Battery Monitor + External I/O");
    Serial.println("Motor Mapping: FL=GPIO4, FR=GPIO5, BR=GPIO6, BL=GPIO3");
    
    // Battery status
    batteryVoltage = getBatteryVoltage();
    Serial.printf("Battery: %.2fV (%d%%) - %s", 
                  batteryVoltage, getBatteryPercentage(), getBatteryStatus().c_str());
    if (batteryVoltage < BATTERY_LOW) {
        Serial.print(" ⚠ LOW BATTERY WARNING");
    }
    Serial.println();
    
    Serial.printf("Armed: %s | Calibrated: %s\n", 
                  armed ? "YES" : "NO", calibrated ? "YES" : "NO");
    Serial.printf("Communication: %s | Last RX: %lums ago\n",
                  communication_established ? "ACTIVE" : "NO CONTROLLER",
                  millis() - lastESPNowReceived);
    // Audio feedback status
    Serial.printf("Audio System: Active (GPIO39/38) | External I/O: GPIO1\n");

    Serial.printf("Throttle: %d PWM (%d%%) | Loop Rate: %.1f Hz\n", 
                  throttle, (throttle*100)/255, 1.0f / dt);
    Serial.printf("Setpoints - Roll: %.1f° | Pitch: %.1f° | Yaw: %.1f°/s\n",
                  rollSetpoint, pitchSetpoint, yawSetpoint);
    Serial.printf("PWM Frequency: %dkHz | Resolution: %d-bit\n", 
                  PWM_FREQ/1000, PWM_RESOLUTION);
    Serial.printf("Angles - Roll: %.2f° | Pitch: %.2f°\n", roll, pitch);
    Serial.printf("Rates - Roll: %.1f°/s | Pitch: %.1f°/s | Yaw: %.1f°/s\n", 
                  rollRate, pitchRate, yawRate);
    Serial.printf("Motor PWM - FL:%d(GPIO4) FR:%d(GPIO5) BL:%d(GPIO3) BR:%d(GPIO6)\n", 
                  motorFL_pwm, motorFR_pwm, motorBL_pwm, motorBR_pwm);
    Serial.printf("Motor Multipliers - FL:%.3f FR:%.3f BL:%.3f BR:%.3f\n",
                  motorFL_multiplier, motorFR_multiplier, motorBL_multiplier, motorBR_multiplier);
    Serial.printf("ESP-NOW Status: Channel 1, Interface STA\n");
    Serial.printf("Controller MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  controllerMAC[0], controllerMAC[1], controllerMAC[2], 
                  controllerMAC[3], controllerMAC[4], controllerMAC[5]);
    if (!armed) {
        Serial.println("*** BRUSHED MOTORS OFF (0% PWM) - SAFE ***");
    } else {
        Serial.println("*** BRUSHED MOTORS SPINNING - CAUTION ***");
    }
    Serial.println("Commands: ARM, DISARM, T+, T-, CAL, TEST, STOP, COMM, LEVEL");
    Serial.println("Attitude: R+10, R-5, P+10, P-5 (roll/pitch setpoints)");
    Serial.println("Motor Calibration: CAL_FL_1.05, CAL_FR_0.95, CAL_BL_1.02, CAL_BR_0.98");
    Serial.println("ESP-NOW: Fixed channel sync for reliable communication");
    Serial.println("Coordinate System: Roll FLIPPED, Pitch FIXED (front faster for +pitch)");
    Serial.println("===============================");
}

// ==================== BUZZER FUNCTIONS ====================
void initBuzzer() {
    pinMode(BUZZER_POS_PIN, OUTPUT);
    pinMode(BUZZER_NEG_PIN, OUTPUT);
    digitalWrite(BUZZER_POS_PIN, LOW);
    digitalWrite(BUZZER_NEG_PIN, LOW);
    Serial.println("Buzzer initialized (GPIO39/38)");
}

void playBuzzerTone(int frequency, int duration) {
    if (frequency <= 0 || duration <= 0) return;
    
    int halfPeriod = 1000000 / frequency / 2;  // microseconds
    unsigned long startTime = millis();
    
    while (millis() - startTime < duration) {
        digitalWrite(BUZZER_POS_PIN, HIGH);
        digitalWrite(BUZZER_NEG_PIN, LOW);
        delayMicroseconds(halfPeriod);
        digitalWrite(BUZZER_POS_PIN, LOW);
        digitalWrite(BUZZER_NEG_PIN, HIGH);
        delayMicroseconds(halfPeriod);
    }
    
    // Turn off buzzer
    digitalWrite(BUZZER_POS_PIN, LOW);
    digitalWrite(BUZZER_NEG_PIN, LOW);
}

void playStartupSequence() {
    Serial.println("Playing startup sequence...");
    playBuzzerTone(800, 150);   // Low tone
    delay(100);
    playBuzzerTone(1200, 150);  // Mid tone  
    delay(100);
    playBuzzerTone(1600, 200);  // High tone
}

void playArmingSequence() {
    Serial.println("BEEP: Armed");
    playBuzzerTone(1000, 200);  // 1kHz for 200ms
    delay(100);
    playBuzzerTone(1500, 200);  // 1.5kHz for 200ms
}

void playDisarmSequence() {
    Serial.println("BEEP: Disarmed");
    playBuzzerTone(800, 300);   // Lower tone for disarm
    delay(100);
    playBuzzerTone(600, 200);   // Even lower tone
}

void playWarningBeep() {
    playBuzzerTone(2000, 100);  // High pitch short beep
    delay(50);
    playBuzzerTone(2000, 100);  // Double beep for urgency
}

void playBatteryWarning() {
    playBuzzerTone(1500, 200);
    delay(100);
    playBuzzerTone(1500, 200);
    delay(100);
    playBuzzerTone(1500, 200);  // Triple beep for battery warning
}

void playCriticalBatteryAlarm() {
    for (int i = 0; i < 5; i++) {
        playBuzzerTone(2500, 100);  // Very high pitch
        delay(100);
    }
}

// ==================== BATTERY MONITORING FUNCTIONS ====================
void initBatteryMonitoring() {
    pinMode(BATTERY_PIN, INPUT);
    batteryVoltage = getBatteryVoltage();
    Serial.printf("Battery monitoring initialized (GPIO2): %.2fV\n", batteryVoltage);
}

float getBatteryVoltage() {
    int reading = analogRead(BATTERY_PIN);
    // Convert ADC reading to voltage (3.3V reference, voltage divider /2)
    float voltage = (reading / 4095.0f) * 3.3f * 2.0f;  // *2 because VBAT/2
    return voltage;
}

String getBatteryStatus() {
    if (batteryVoltage >= BATTERY_GOOD) return "GOOD";
    else if (batteryVoltage >= BATTERY_LOW) return "MEDIUM";
    else if (batteryVoltage >= BATTERY_CRITICAL) return "LOW";
    else return "CRITICAL";
}

int getBatteryPercentage() {
    // Rough LiPo percentage estimation (2S pack: 6.0V - 8.4V range)
    float percentage = ((batteryVoltage - 6.0f) / (8.4f - 6.0f)) * 100.0f;
    return constrain((int)percentage, 0, 100);
}

bool checkBatteryLevel() {
    batteryVoltage = getBatteryVoltage();
    bool batteryOK = true;
    
    if (batteryVoltage < BATTERY_CRITICAL) {
        Serial.printf("*** CRITICAL BATTERY: %.2fV ***\n", batteryVoltage);
        playCriticalBatteryAlarm();
        batteryOK = false;
        
        // Auto-disarm if armed
        if (armed) {
            Serial.println("AUTO-DISARMING: Critical battery level");
            armed = false;
            setAllMotorsOff();
            resetPIDs();
        }
    } 
    else if (batteryVoltage < BATTERY_LOW) {
        // Only warn every 10 seconds to avoid spam
        if (millis() - lastBatteryWarning > 10000) {
            Serial.printf("Low battery warning: %.2fV (%d%%)\n", 
                         batteryVoltage, getBatteryPercentage());
            playBatteryWarning();
            lastBatteryWarning = millis();
            batteryLowWarningActive = true;
        }
    }
    else {
        batteryLowWarningActive = false;  // Reset warning flag when battery is OK
    }
    
    return batteryOK;
}

// ==================== EXTERNAL I/O FUNCTIONS ====================

void initExternalIO() {
    pinMode(EXT_IO_PIN, OUTPUT);
    digitalWrite(EXT_IO_PIN, LOW);
    Serial.println("External I/O initialized (GPIO1)");
}

void setExternalLED(bool state) {
    digitalWrite(EXT_IO_PIN, state ? HIGH : LOW);
}

// ==================== ENHANCED STATUS FUNCTIONS ====================

void updateEnhancedStatus() {
    // Update battery voltage in response data
    responseData.battery_voltage = batteryVoltage;
    
    // Enhanced LED status with battery indication
    if (batteryVoltage < BATTERY_CRITICAL) {
        setStatusLED(255, 0, 0);  // Red for critical battery
    } 
    else if (batteryVoltage < BATTERY_LOW) {
        // Blinking orange for low battery
        static bool blinkState = false;
        static unsigned long lastBlink = 0;
        if (millis() - lastBlink > 500) {
            blinkState = !blinkState;
            setStatusLED(blinkState ? 255 : 0, blinkState ? 165 : 0, 0);
            lastBlink = millis();
        }
    }
    else if (armed) {
        setStatusLED(0, 0, 255);  // Blue when flying
    } 
    else if (communication_established) {
        setStatusLED(0, 255, 0);  // Green when ready
    }
    else {
        setStatusLED(255, 255, 0);  // Yellow when no communication
    }
}
