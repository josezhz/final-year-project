#include <WiFi.h>
#include <Wire.h>
#include <esp_now.h>
#include <cstring>
#include <cstdlib>

constexpr unsigned long USB_BAUD_RATE = 115200;
constexpr size_t MAX_PAYLOAD_LENGTH = 251;
constexpr unsigned long SERIAL_WAIT_MS = 2000;
constexpr unsigned long STATUS_PRINT_INTERVAL_MS = 1000;
constexpr unsigned long COMMAND_TIMEOUT_MS = 250;
constexpr unsigned long ARM_RAMP_MS = 450;
constexpr unsigned long IMU_REINIT_RETRY_MS = 1000;
constexpr unsigned long IMU_ERROR_LOG_INTERVAL_MS = 1000;

constexpr uint8_t I2C_SDA_PIN = 11;
constexpr uint8_t I2C_SCL_PIN = 10;
constexpr uint8_t MPU6050_ADDRESS = 0x68;
constexpr uint32_t I2C_FREQUENCY_HZ = 100000;
constexpr uint16_t I2C_TIMEOUT_MS = 20;
constexpr uint8_t I2C_TRANSACTION_RETRIES = 3;
constexpr uint8_t IMU_FAILURES_BEFORE_REINIT = 5;
constexpr uint8_t MOTOR_COUNT = 4;
constexpr uint8_t MOTOR_PINS[MOTOR_COUNT] = {5, 6, 3, 4};
constexpr int PWM_FREQUENCY_HZ = 20000;
constexpr int PWM_RESOLUTION_BITS = 10;
constexpr int PWM_MAX_DUTY = (1 << PWM_RESOLUTION_BITS) - 1;

constexpr float ACCEL_SCALE_LSB_PER_G = 16384.0f;
constexpr float GYRO_SCALE_LSB_PER_DEG_PER_SEC = 65.5f;
constexpr float RAD_TO_DEG_F = 57.2957795f;
constexpr float DEG_TO_RAD_F = 0.0174532925f;
constexpr float IMU_ANGLE_KALMAN_PROCESS_NOISE = 0.001f;
constexpr float IMU_ANGLE_KALMAN_BIAS_NOISE = 0.003f;
constexpr float IMU_ANGLE_KALMAN_MEASUREMENT_NOISE = 0.03f;
constexpr float IMU_RATE_KALMAN_PROCESS_NOISE = 1.5f;
constexpr float IMU_RATE_KALMAN_MEASUREMENT_NOISE = 6.0f;
constexpr float MAX_WORLD_XY_VELOCITY_MPS = 0.8f;
constexpr float MAX_WORLD_Z_VELOCITY_MPS = 0.5f;
// Observed IMU mounting on this airframe:
// - nose down was reported as +roll
// - CCW roll (viewed from the back) was reported as +pitch
// Remap sensor axes into the physical body frame so:
// - +pitch = nose up
// - +roll = clockwise when viewed from the back (right side down)

// X-quad mixer using the confirmed board order:
// M1 = front-right, M2 = back-right, M3 = back-left, M4 = front-left.
// If yaw response is inverted during bench testing, flip MOTOR_YAW_SIGN.
constexpr float MOTOR_ROLL_SIGN[MOTOR_COUNT] = {-1.0f, -1.0f, 1.0f, 1.0f};
// Positive pitch correction means nose up, so front motors rise and rear motors fall.
constexpr float MOTOR_PITCH_SIGN[MOTOR_COUNT] = {1.0f, -1.0f, -1.0f, 1.0f};
constexpr float MOTOR_YAW_SIGN[MOTOR_COUNT] = {-1.0f, 1.0f, -1.0f, 1.0f};

struct EspNowMessage {
  char payload[MAX_PAYLOAD_LENGTH];
};

struct PidGains {
  float kp;
  float ki;
  float kd;
};

struct PidState {
  float integral;
  float previousError;
  bool hasPreviousError;
};

struct ScalarKalmanFilter {
  float value;
  float errorCovariance;
  float processNoise;
  float measurementNoise;
  bool initialized;
};

struct AngleKalmanFilter {
  float angle;
  float bias;
  float rate;
  float errorCovariance00;
  float errorCovariance01;
  float errorCovariance10;
  float errorCovariance11;
  float angleProcessNoise;
  float biasProcessNoise;
  float measurementNoise;
  bool initialized;
};

struct FlightCommand {
  bool armed;
  bool spatialValid;
  float position[3];
  float velocity[3];
  float rotation[3];
  float target[4];
  PidGains xyPos;
  PidGains zPos;
  PidGains yawPos;
  PidGains xyVel;
  PidGains zVel;
  PidGains roll;
  PidGains pitch;
  PidGains yawRate;
  float hoverThrottle;
  float minThrottle;
  float maxThrottle;
  float maxTiltDeg;
  float maxYawRateDeg;
  uint32_t levelCalibrationSequence;
};

struct ImuState {
  float rawRollDeg;
  float rawPitchDeg;
  float rollDeg;
  float pitchDeg;
  float yawDeg;
  float gyroXDegPerSec;
  float gyroYDegPerSec;
  float gyroZDegPerSec;
  float levelRollOffsetDeg;
  float levelPitchOffsetDeg;
  bool ready;
};

portMUX_TYPE payloadMux = portMUX_INITIALIZER_UNLOCKED;
char pendingPayload[MAX_PAYLOAD_LENGTH] = {};
volatile bool pendingPayloadReady = false;

FlightCommand flightCommand = {};
ImuState imuState = {};

PidState posXPid = {};
PidState posYPid = {};
PidState posZPid = {};
PidState yawOuterPid = {};
PidState xVelPid = {};
PidState yVelPid = {};
PidState zVelPid = {};
PidState rollPid = {};
PidState pitchPid = {};
PidState yawRatePid = {};
AngleKalmanFilter rollAngleFilter = {};
AngleKalmanFilter pitchAngleFilter = {};
ScalarKalmanFilter yawRateFilter = {};

float motorOutputs[MOTOR_COUNT] = {};
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;
uint32_t lastHandledLevelCalibrationSequence = 0;
uint8_t consecutiveImuFailures = 0;

unsigned long lastCommandRxMs = 0;
unsigned long armStartMs = 0;
unsigned long lastStatusPrintMs = 0;
unsigned long nextImuInitAttemptMs = 0;
unsigned long lastImuErrorLogMs = 0;
bool motorsEnabled = false;
bool imuConfigured = false;

float clampf(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

float wrapDegrees(float angle) {
  while (angle > 180.0f) {
    angle -= 360.0f;
  }
  while (angle < -180.0f) {
    angle += 360.0f;
  }
  return angle;
}

PidGains makePidGains(float kp, float ki, float kd) {
  PidGains gains = {kp, ki, kd};
  return gains;
}

void configureScalarKalmanFilter(
  ScalarKalmanFilter &filter,
  float processNoise,
  float measurementNoise
) {
  filter.value = 0.0f;
  filter.errorCovariance = 1.0f;
  filter.processNoise = processNoise;
  filter.measurementNoise = measurementNoise;
  filter.initialized = false;
}

float updateScalarKalmanFilter(ScalarKalmanFilter &filter, float measurement) {
  if (!filter.initialized) {
    filter.value = measurement;
    filter.errorCovariance = 1.0f;
    filter.initialized = true;
    return filter.value;
  }

  filter.errorCovariance += filter.processNoise;
  const float innovationCovariance = filter.errorCovariance + filter.measurementNoise;
  if (innovationCovariance <= 0.0f) {
    return filter.value;
  }

  const float kalmanGain = filter.errorCovariance / innovationCovariance;
  filter.value += kalmanGain * (measurement - filter.value);
  filter.errorCovariance *= (1.0f - kalmanGain);
  return filter.value;
}

void configureAngleKalmanFilter(
  AngleKalmanFilter &filter,
  float angleProcessNoise,
  float biasProcessNoise,
  float measurementNoise
) {
  filter.angle = 0.0f;
  filter.bias = 0.0f;
  filter.rate = 0.0f;
  filter.errorCovariance00 = 1.0f;
  filter.errorCovariance01 = 0.0f;
  filter.errorCovariance10 = 0.0f;
  filter.errorCovariance11 = 1.0f;
  filter.angleProcessNoise = angleProcessNoise;
  filter.biasProcessNoise = biasProcessNoise;
  filter.measurementNoise = measurementNoise;
  filter.initialized = false;
}

float updateAngleKalmanFilter(
  AngleKalmanFilter &filter,
  float measuredAngleDeg,
  float measuredRateDegPerSec,
  float dtSeconds
) {
  if (!filter.initialized) {
    filter.angle = measuredAngleDeg;
    filter.bias = 0.0f;
    filter.rate = measuredRateDegPerSec;
    filter.errorCovariance00 = 1.0f;
    filter.errorCovariance01 = 0.0f;
    filter.errorCovariance10 = 0.0f;
    filter.errorCovariance11 = 1.0f;
    filter.initialized = true;
    return filter.angle;
  }

  if (dtSeconds <= 0.0f) {
    return filter.angle;
  }

  filter.rate = measuredRateDegPerSec - filter.bias;
  filter.angle += dtSeconds * filter.rate;

  filter.errorCovariance00 += dtSeconds * (
    (dtSeconds * filter.errorCovariance11)
    - filter.errorCovariance01
    - filter.errorCovariance10
    + filter.angleProcessNoise
  );
  filter.errorCovariance01 -= dtSeconds * filter.errorCovariance11;
  filter.errorCovariance10 -= dtSeconds * filter.errorCovariance11;
  filter.errorCovariance11 += filter.biasProcessNoise * dtSeconds;

  const float innovation = measuredAngleDeg - filter.angle;
  const float innovationCovariance = filter.errorCovariance00 + filter.measurementNoise;
  if (innovationCovariance <= 0.0f) {
    return filter.angle;
  }

  const float kalmanGain0 = filter.errorCovariance00 / innovationCovariance;
  const float kalmanGain1 = filter.errorCovariance10 / innovationCovariance;
  const float errorCovariance00 = filter.errorCovariance00;
  const float errorCovariance01 = filter.errorCovariance01;

  filter.angle += kalmanGain0 * innovation;
  filter.bias += kalmanGain1 * innovation;
  filter.errorCovariance00 -= kalmanGain0 * errorCovariance00;
  filter.errorCovariance01 -= kalmanGain0 * errorCovariance01;
  filter.errorCovariance10 -= kalmanGain1 * errorCovariance00;
  filter.errorCovariance11 -= kalmanGain1 * errorCovariance01;

  return filter.angle;
}

void resetImuFilters() {
  configureAngleKalmanFilter(
    rollAngleFilter,
    IMU_ANGLE_KALMAN_PROCESS_NOISE,
    IMU_ANGLE_KALMAN_BIAS_NOISE,
    IMU_ANGLE_KALMAN_MEASUREMENT_NOISE
  );
  configureAngleKalmanFilter(
    pitchAngleFilter,
    IMU_ANGLE_KALMAN_PROCESS_NOISE,
    IMU_ANGLE_KALMAN_BIAS_NOISE,
    IMU_ANGLE_KALMAN_MEASUREMENT_NOISE
  );
  configureScalarKalmanFilter(
    yawRateFilter,
    IMU_RATE_KALMAN_PROCESS_NOISE,
    IMU_RATE_KALMAN_MEASUREMENT_NOISE
  );
}

FlightCommand makeDefaultFlightCommand() {
  FlightCommand command = {};
  command.armed = false;
  command.spatialValid = false;
  command.target[0] = 0.0f;
  command.target[1] = 0.0f;
  command.target[2] = 0.25f;
  command.target[3] = 0.0f;  // Hold the nose on the mocap world +X axis by default.
  command.xyPos = makePidGains(1.0f, 0.0f, 0.0f);
  command.zPos = makePidGains(1.5f, 0.0f, 0.0f);
  command.yawPos = makePidGains(0.3f, 0.1f, 0.05f);
  command.xyVel = makePidGains(0.2f, 0.03f, 0.05f);
  command.zVel = makePidGains(0.3f, 0.1f, 0.05f);
  command.roll = makePidGains(0.022f, 0.0f, 0.0014f);
  command.pitch = makePidGains(0.022f, 0.0f, 0.0014f);
  command.yawRate = makePidGains(0.006f, 0.0f, 0.0f);
  command.hoverThrottle = 0.36f;
  command.minThrottle = 0.18f;
  command.maxThrottle = 0.82f;
  command.maxTiltDeg = 12.0f;
  command.maxYawRateDeg = 120.0f;
  command.levelCalibrationSequence = 0;
  return command;
}

void resetPidState(PidState &state) {
  state.integral = 0.0f;
  state.previousError = 0.0f;
  state.hasPreviousError = false;
}

void resetAllPidStates() {
  resetPidState(posXPid);
  resetPidState(posYPid);
  resetPidState(posZPid);
  resetPidState(yawOuterPid);
  resetPidState(xVelPid);
  resetPidState(yVelPid);
  resetPidState(zVelPid);
  resetPidState(rollPid);
  resetPidState(pitchPid);
  resetPidState(yawRatePid);
}

float runPid(const PidGains &gains, PidState &state, float error, float dtSeconds, float integralLimit) {
  if (dtSeconds <= 0.0f) {
    return gains.kp * error;
  }

  state.integral += error * dtSeconds;
  state.integral = clampf(state.integral, -integralLimit, integralLimit);

  float derivative = 0.0f;
  if (state.hasPreviousError) {
    derivative = (error - state.previousError) / dtSeconds;
  }

  state.previousError = error;
  state.hasPreviousError = true;

  return (gains.kp * error) + (gains.ki * state.integral) + (gains.kd * derivative);
}

float throttleFromNormalizedCommand(float command) {
  command = clampf(command, -1.0f, 1.0f);
  if (command >= 0.0f) {
    return flightCommand.hoverThrottle
      + (command * (flightCommand.maxThrottle - flightCommand.hoverThrottle));
  }
  return flightCommand.hoverThrottle
    + (command * (flightCommand.hoverThrottle - flightCommand.minThrottle));
}

bool parseBoolFlag(const char *json, const char *key, bool &value) {
  const char *cursor = strstr(json, key);
  if (cursor == nullptr) {
    return false;
  }
  cursor += strlen(key);

  if (*cursor == '1') {
    value = true;
    return true;
  }
  if (*cursor == '0') {
    value = false;
    return true;
  }
  if (strncmp(cursor, "true", 4) == 0) {
    value = true;
    return true;
  }
  if (strncmp(cursor, "false", 5) == 0) {
    value = false;
    return true;
  }
  return false;
}

bool parseFloatArray(const char *json, const char *key, float *values, size_t count) {
  const char *cursor = strstr(json, key);
  if (cursor == nullptr) {
    return false;
  }

  cursor = strchr(cursor, '[');
  if (cursor == nullptr) {
    return false;
  }
  ++cursor;

  for (size_t index = 0; index < count; ++index) {
    char *parseEnd = nullptr;
    values[index] = strtof(cursor, &parseEnd);
    if (parseEnd == cursor) {
      return false;
    }
    cursor = parseEnd;

    if (index + 1 < count) {
      cursor = strchr(cursor, ',');
      if (cursor == nullptr) {
        return false;
      }
      ++cursor;
    }
  }

  cursor = strchr(cursor, ']');
  return cursor != nullptr;
}

bool parseBoolFlagEither(const char *json, const char *primaryKey, const char *fallbackKey, bool &value) {
  return parseBoolFlag(json, primaryKey, value) || parseBoolFlag(json, fallbackKey, value);
}

bool parseUnsignedValue(const char *json, const char *key, uint32_t &value) {
  const char *cursor = strstr(json, key);
  if (cursor == nullptr) {
    return false;
  }
  cursor += strlen(key);

  char *parseEnd = nullptr;
  const unsigned long parsedValue = strtoul(cursor, &parseEnd, 10);
  if (parseEnd == cursor) {
    return false;
  }

  value = static_cast<uint32_t>(parsedValue);
  return true;
}

bool parseFloatArrayEither(
  const char *json,
  const char *primaryKey,
  const char *fallbackKey,
  float *values,
  size_t count
) {
  return parseFloatArray(json, primaryKey, values, count) || parseFloatArray(json, fallbackKey, values, count);
}

PidGains gainsFromTriplet(const float values[3]) {
  return makePidGains(values[0], values[1], values[2]);
}

bool parsePidProfilePayload(const char *payload, FlightCommand &parsed) {
  float pidBundle[24] = {};
  float values3[3] = {};

  if (parseFloatArray(payload, "\"u\":", pidBundle, 24)) {
    parsed.xyPos = makePidGains(pidBundle[0], pidBundle[1], pidBundle[2]);
    parsed.zPos = makePidGains(pidBundle[3], pidBundle[4], pidBundle[5]);
    parsed.yawPos = makePidGains(pidBundle[6], pidBundle[7], pidBundle[8]);
    parsed.xyVel = makePidGains(pidBundle[9], pidBundle[10], pidBundle[11]);
    parsed.zVel = makePidGains(pidBundle[12], pidBundle[13], pidBundle[14]);
    parsed.roll = makePidGains(pidBundle[15], pidBundle[16], pidBundle[17]);
    parsed.pitch = makePidGains(pidBundle[18], pidBundle[19], pidBundle[20]);
    parsed.yawRate = makePidGains(pidBundle[21], pidBundle[22], pidBundle[23]);
    return true;
  }

  if (!parseFloatArray(payload, "\"xp\":", values3, 3)) {
    return false;
  }
  parsed.xyPos = gainsFromTriplet(values3);

  if (!parseFloatArray(payload, "\"zp\":", values3, 3)) {
    return false;
  }
  parsed.zPos = gainsFromTriplet(values3);

  if (!parseFloatArray(payload, "\"yp\":", values3, 3)) {
    return false;
  }
  parsed.yawPos = gainsFromTriplet(values3);

  if (!parseFloatArray(payload, "\"xv\":", values3, 3)) {
    return false;
  }
  parsed.xyVel = gainsFromTriplet(values3);

  if (!parseFloatArray(payload, "\"zv\":", values3, 3)) {
    return false;
  }
  parsed.zVel = gainsFromTriplet(values3);

  if (!parseFloatArray(payload, "\"rp\":", values3, 3)) {
    return false;
  }
  parsed.roll = gainsFromTriplet(values3);

  if (!parseFloatArray(payload, "\"pp\":", values3, 3)) {
    return false;
  }
  parsed.pitch = gainsFromTriplet(values3);

  if (!parseFloatArray(payload, "\"yr\":", values3, 3)) {
    return false;
  }
  parsed.yawRate = gainsFromTriplet(values3);
  return true;
}

bool parseFlightCommandPayload(const char *payload, FlightCommand &nextCommand) {
  FlightCommand parsed = flightCommand;
  float values4[4] = {};
  float limits[5] = {};

  parsed.levelCalibrationSequence = 0;

  if (!parseBoolFlagEither(payload, "\"a\":", "\"arm\":", parsed.armed)) {
    return false;
  }
  if (!parseBoolFlagEither(payload, "\"o\":", "\"ok\":", parsed.spatialValid)) {
    return false;
  }
  if (!parseFloatArray(payload, "\"p\":", parsed.position, 3)) {
    return false;
  }
  parsed.velocity[0] = 0.0f;
  parsed.velocity[1] = 0.0f;
  parsed.velocity[2] = 0.0f;
  parseFloatArrayEither(payload, "\"d\":", "\"vel\":", parsed.velocity, 3);
  if (!parseFloatArray(payload, "\"r\":", parsed.rotation, 3)) {
    return false;
  }
  if (!parseFloatArrayEither(payload, "\"g\":", "\"tg\":", values4, 4)) {
    return false;
  }
  memcpy(parsed.target, values4, sizeof(values4));

  if (!parsePidProfilePayload(payload, parsed)) {
    return false;
  }

  if (!parseFloatArrayEither(payload, "\"m\":", "\"lm\":", limits, 5)) {
    return false;
  }

  parsed.minThrottle = clampf(limits[1], 0.0f, 1.0f);
  parsed.hoverThrottle = clampf(limits[0], parsed.minThrottle, 1.0f);
  parsed.maxThrottle = clampf(limits[2], parsed.hoverThrottle, 1.0f);
  parsed.maxTiltDeg = clampf(limits[3], 1.0f, 45.0f);
  parsed.maxYawRateDeg = clampf(limits[4], 1.0f, 360.0f);
  parseUnsignedValue(payload, "\"l\":", parsed.levelCalibrationSequence);

  nextCommand = parsed;
  return true;
}

void onDataReceived(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int dataLen) {
  (void)recvInfo;

  const size_t copyLength = min(static_cast<size_t>(dataLen), sizeof(pendingPayload) - 1);
  portENTER_CRITICAL_ISR(&payloadMux);
  memcpy(pendingPayload, data, copyLength);
  pendingPayload[copyLength] = '\0';
  pendingPayloadReady = true;
  portEXIT_CRITICAL_ISR(&payloadMux);
}

bool pullPendingPayload(char *buffer, size_t bufferSize) {
  bool hasPayload = false;
  portENTER_CRITICAL(&payloadMux);
  if (pendingPayloadReady) {
    strncpy(buffer, pendingPayload, bufferSize - 1);
    buffer[bufferSize - 1] = '\0';
    pendingPayloadReady = false;
    hasPayload = true;
  }
  portEXIT_CRITICAL(&payloadMux);
  return hasPayload;
}

bool initEspNowReceiver() {
  Serial.println("Configuring WiFi station mode...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.println("Initialising ESP-NOW receiver...");
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed.");
    return false;
  }

  esp_now_register_recv_cb(onDataReceived);
  Serial.println("ESP-NOW receive callback registered.");
  return true;
}

void setupMotors() {
  for (uint8_t index = 0; index < MOTOR_COUNT; ++index) {
    ledcAttach(MOTOR_PINS[index], PWM_FREQUENCY_HZ, PWM_RESOLUTION_BITS);
    ledcWrite(MOTOR_PINS[index], 0);
    motorOutputs[index] = 0.0f;
  }
}

void writeMotorOutputs(const float outputs[MOTOR_COUNT]) {
  for (uint8_t index = 0; index < MOTOR_COUNT; ++index) {
    motorOutputs[index] = clampf(outputs[index], 0.0f, 1.0f);
    const uint32_t duty = static_cast<uint32_t>(motorOutputs[index] * PWM_MAX_DUTY);
    ledcWrite(MOTOR_PINS[index], duty);
  }
}

void stopMotors() {
  float stopped[MOTOR_COUNT] = {};
  writeMotorOutputs(stopped);
  motorsEnabled = false;
  resetAllPidStates();
}

void logImuError(const char *message) {
  if ((millis() - lastImuErrorLogMs) < IMU_ERROR_LOG_INTERVAL_MS) {
    return;
  }
  lastImuErrorLogMs = millis();
  Serial.println(message);
}

void initI2cBus() {
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, INPUT_PULLUP);
  Wire.end();
  delay(2);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQUENCY_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);
  delay(10);
}

bool writeMpuRegister(uint8_t reg, uint8_t value) {
  for (uint8_t attempt = 0; attempt < I2C_TRANSACTION_RETRIES; ++attempt) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(reg);
    Wire.write(value);
    if (Wire.endTransmission() == 0) {
      return true;
    }

    if (attempt + 1 < I2C_TRANSACTION_RETRIES) {
      delay(2);
      initI2cBus();
    }
  }
  return false;
}

bool readMpuRegisters(uint8_t startRegister, uint8_t *buffer, size_t length) {
  for (uint8_t attempt = 0; attempt < I2C_TRANSACTION_RETRIES; ++attempt) {
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(startRegister);
    if (Wire.endTransmission(false) == 0) {
      const size_t bytesRead = Wire.requestFrom(
        MPU6050_ADDRESS,
        static_cast<uint8_t>(length),
        static_cast<uint8_t>(true)
      );
      if (bytesRead == length) {
        for (size_t index = 0; index < length; ++index) {
          buffer[index] = Wire.read();
        }
        return true;
      }
    }

    if (attempt + 1 < I2C_TRANSACTION_RETRIES) {
      delay(2);
      initI2cBus();
    }
  }
  return false;
}

bool initMpu6050() {
  initI2cBus();

  uint8_t whoAmI = 0;
  if (!readMpuRegisters(0x75, &whoAmI, 1)) {
    Serial.println("Failed to read MPU6050 WHO_AM_I register.");
    return false;
  }

  if (!writeMpuRegister(0x6B, 0x00)) {
    Serial.println("Failed to wake MPU6050.");
    return false;
  }
  delay(20);

  if (!writeMpuRegister(0x1A, 0x03) || !writeMpuRegister(0x1B, 0x08) || !writeMpuRegister(0x1C, 0x00)) {
    Serial.println("Failed to configure MPU6050 registers.");
    return false;
  }

  Serial.print("MPU6050 ready. WHO_AM_I = 0x");
  Serial.println(whoAmI, HEX);
  return true;
}

bool bringUpMpu6050() {
  if (!initMpu6050()) {
    logImuError("MPU6050 init failed. Check 3.3V, GND, SDA, SCL, and pull-ups.");
    imuConfigured = false;
    return false;
  }
  if (!calibrateGyroBias()) {
    logImuError("MPU6050 gyro calibration failed. Retrying.");
    imuConfigured = false;
    return false;
  }

  imuConfigured = true;
  consecutiveImuFailures = 0;
  imuState.ready = false;
  imuState.rawRollDeg = 0.0f;
  imuState.rawPitchDeg = 0.0f;
  imuState.rollDeg = 0.0f;
  imuState.pitchDeg = 0.0f;
  imuState.yawDeg = 0.0f;
  imuState.gyroXDegPerSec = 0.0f;
  imuState.gyroYDegPerSec = 0.0f;
  imuState.gyroZDegPerSec = 0.0f;
  resetImuFilters();
  Serial.printf(
    "MPU6050 online on SDA=%u SCL=%u at %lu Hz.\n",
    static_cast<unsigned>(I2C_SDA_PIN),
    static_cast<unsigned>(I2C_SCL_PIN),
    static_cast<unsigned long>(I2C_FREQUENCY_HZ)
  );
  return true;
}

bool ensureMpu6050Ready() {
  if (imuConfigured) {
    return true;
  }
  if (millis() < nextImuInitAttemptMs) {
    return false;
  }

  nextImuInitAttemptMs = millis() + IMU_REINIT_RETRY_MS;
  Serial.println("Attempting MPU6050 bring-up...");
  return bringUpMpu6050();
}

bool calibrateGyroBias() {
  constexpr int SAMPLE_COUNT = 500;
  float sumX = 0.0f;
  float sumY = 0.0f;
  float sumZ = 0.0f;
  uint8_t rawData[14] = {};

  Serial.println("Calibrating gyro bias. Keep the drone still...");
  delay(250);

  for (int sample = 0; sample < SAMPLE_COUNT; ++sample) {
    if (!readMpuRegisters(0x3B, rawData, sizeof(rawData))) {
      Serial.println("Gyro calibration failed while reading MPU6050.");
      return false;
    }

    const int16_t gyroXRaw = static_cast<int16_t>((rawData[8] << 8) | rawData[9]);
    const int16_t gyroYRaw = static_cast<int16_t>((rawData[10] << 8) | rawData[11]);
    const int16_t gyroZRaw = static_cast<int16_t>((rawData[12] << 8) | rawData[13]);

    sumX += static_cast<float>(gyroXRaw) / GYRO_SCALE_LSB_PER_DEG_PER_SEC;
    sumY += static_cast<float>(gyroYRaw) / GYRO_SCALE_LSB_PER_DEG_PER_SEC;
    sumZ += static_cast<float>(gyroZRaw) / GYRO_SCALE_LSB_PER_DEG_PER_SEC;
    delay(3);
  }

  gyroBiasX = sumX / SAMPLE_COUNT;
  gyroBiasY = sumY / SAMPLE_COUNT;
  gyroBiasZ = sumZ / SAMPLE_COUNT;

  Serial.printf(
    "Gyro bias: x=%.3f y=%.3f z=%.3f deg/s\n",
    gyroBiasX,
    gyroBiasY,
    gyroBiasZ
  );
  return true;
}

bool updateImuEstimate(float dtSeconds) {
  uint8_t rawData[14] = {};
  if (!readMpuRegisters(0x3B, rawData, sizeof(rawData))) {
    if (consecutiveImuFailures < 255) {
      ++consecutiveImuFailures;
    }
    if (consecutiveImuFailures >= IMU_FAILURES_BEFORE_REINIT) {
      imuConfigured = false;
      imuState.ready = false;
      nextImuInitAttemptMs = 0;
      logImuError("Lost MPU6050 I2C link. Reinitialising at 100 kHz.");
    }
    return false;
  }
  consecutiveImuFailures = 0;

  const int16_t accelXRaw = static_cast<int16_t>((rawData[0] << 8) | rawData[1]);
  const int16_t accelYRaw = static_cast<int16_t>((rawData[2] << 8) | rawData[3]);
  const int16_t accelZRaw = static_cast<int16_t>((rawData[4] << 8) | rawData[5]);
  const int16_t gyroXRaw = static_cast<int16_t>((rawData[8] << 8) | rawData[9]);
  const int16_t gyroYRaw = static_cast<int16_t>((rawData[10] << 8) | rawData[11]);
  const int16_t gyroZRaw = static_cast<int16_t>((rawData[12] << 8) | rawData[13]);

  const float accelX = static_cast<float>(accelXRaw) / ACCEL_SCALE_LSB_PER_G;
  const float accelY = static_cast<float>(accelYRaw) / ACCEL_SCALE_LSB_PER_G;
  const float accelZ = static_cast<float>(accelZRaw) / ACCEL_SCALE_LSB_PER_G;

  const float sensorGyroXDegPerSec = (static_cast<float>(gyroXRaw) / GYRO_SCALE_LSB_PER_DEG_PER_SEC) - gyroBiasX;
  const float sensorGyroYDegPerSec = (static_cast<float>(gyroYRaw) / GYRO_SCALE_LSB_PER_DEG_PER_SEC) - gyroBiasY;
  const float sensorGyroZDegPerSec = (static_cast<float>(gyroZRaw) / GYRO_SCALE_LSB_PER_DEG_PER_SEC) - gyroBiasZ;

  // Sensor-frame to body-frame remap:
  // sensor +X -> body +right
  // sensor +Y -> body +forward
  // sensor +Z -> body +up/down unchanged
  const float bodyForwardAccel = accelY;
  const float bodyRightAccel = accelX;
  const float bodyVerticalAccel = accelZ;

  const float bodyGyroXDegPerSec = -sensorGyroYDegPerSec;
  const float bodyGyroYDegPerSec = -sensorGyroXDegPerSec;
  const float bodyGyroZDegPerSec = sensorGyroZDegPerSec;

  const float rollAccDeg = atan2f(bodyRightAccel, bodyVerticalAccel) * RAD_TO_DEG_F;
  const float pitchAccDeg = atan2f(
    -bodyForwardAccel,
    sqrtf((bodyRightAccel * bodyRightAccel) + (bodyVerticalAccel * bodyVerticalAccel))
  ) * RAD_TO_DEG_F;

  imuState.gyroXDegPerSec = bodyGyroXDegPerSec;
  imuState.gyroYDegPerSec = bodyGyroYDegPerSec;
  imuState.gyroZDegPerSec = updateScalarKalmanFilter(yawRateFilter, bodyGyroZDegPerSec);

  imuState.rawRollDeg = updateAngleKalmanFilter(
    rollAngleFilter,
    rollAccDeg,
    bodyGyroXDegPerSec,
    dtSeconds
  );
  imuState.rawPitchDeg = updateAngleKalmanFilter(
    pitchAngleFilter,
    pitchAccDeg,
    bodyGyroYDegPerSec,
    dtSeconds
  );

  if (!imuState.ready) {
    imuState.yawDeg = 0.0f;
    imuState.ready = true;
  } else {
    imuState.yawDeg = wrapDegrees(imuState.yawDeg + (imuState.gyroZDegPerSec * dtSeconds));
  }

  imuState.rollDeg = imuState.rawRollDeg - imuState.levelRollOffsetDeg;
  imuState.pitchDeg = imuState.rawPitchDeg - imuState.levelPitchOffsetDeg;

  return true;
}

void processLevelCalibrationRequest() {
  const uint32_t sequence = flightCommand.levelCalibrationSequence;
  if (sequence == 0 || sequence == lastHandledLevelCalibrationSequence) {
    return;
  }

  if (!imuState.ready) {
    Serial.println("Delaying IMU level calibration: IMU estimate is not ready yet.");
    return;
  }

  lastHandledLevelCalibrationSequence = sequence;
  if (flightCommand.armed || motorsEnabled) {
    Serial.println("Ignoring IMU level calibration request while the drone is armed.");
    return;
  }

  imuState.levelRollOffsetDeg = imuState.rawRollDeg;
  imuState.levelPitchOffsetDeg = imuState.rawPitchDeg;
  imuState.rollDeg = 0.0f;
  imuState.pitchDeg = 0.0f;
  resetAllPidStates();
  Serial.printf(
    "IMU level reference captured: rollOffset=%.2f pitchOffset=%.2f seq=%lu\n",
    imuState.levelRollOffsetDeg,
    imuState.levelPitchOffsetDeg,
    static_cast<unsigned long>(sequence)
  );
}

void normalizeMotorOutputs(float outputs[MOTOR_COUNT]) {
  float minOutput = outputs[0];
  float maxOutput = outputs[0];

  for (uint8_t index = 1; index < MOTOR_COUNT; ++index) {
    if (outputs[index] < minOutput) {
      minOutput = outputs[index];
    }
    if (outputs[index] > maxOutput) {
      maxOutput = outputs[index];
    }
  }

  if (maxOutput > 1.0f) {
    const float excess = maxOutput - 1.0f;
    for (uint8_t index = 0; index < MOTOR_COUNT; ++index) {
      outputs[index] -= excess;
    }
    minOutput -= excess;
  }

  if (minOutput < 0.0f) {
    const float deficit = -minOutput;
    for (uint8_t index = 0; index < MOTOR_COUNT; ++index) {
      outputs[index] += deficit;
    }
  }

  for (uint8_t index = 0; index < MOTOR_COUNT; ++index) {
    outputs[index] = clampf(outputs[index], 0.0f, 1.0f);
  }
}

void runHoverController(float dtSeconds) {
  const bool commandFresh = (millis() - lastCommandRxMs) <= COMMAND_TIMEOUT_MS;
  const bool controlReady = commandFresh && flightCommand.armed && flightCommand.spatialValid && imuState.ready;

  if (!controlReady) {
    stopMotors();
    return;
  }

  if (!motorsEnabled) {
    motorsEnabled = true;
    armStartMs = millis();
    resetAllPidStates();
  }

  const float xError = flightCommand.target[0] - flightCommand.position[0];
  const float yError = flightCommand.target[1] - flightCommand.position[1];
  const float zError = flightCommand.target[2] - flightCommand.position[2];
  const float yawError = wrapDegrees(flightCommand.target[3] - flightCommand.rotation[0]);
  const float worldXVelocity = flightCommand.velocity[0];
  const float worldYVelocity = flightCommand.velocity[1];
  const float worldZVelocity = flightCommand.velocity[2];

  const float desiredWorldXVelocity = clampf(
    runPid(
      flightCommand.xyPos,
      posXPid,
      xError,
      dtSeconds,
      MAX_WORLD_XY_VELOCITY_MPS
    ),
    -MAX_WORLD_XY_VELOCITY_MPS,
    MAX_WORLD_XY_VELOCITY_MPS
  );
  const float desiredWorldYVelocity = clampf(
    runPid(
      flightCommand.xyPos,
      posYPid,
      yError,
      dtSeconds,
      MAX_WORLD_XY_VELOCITY_MPS
    ),
    -MAX_WORLD_XY_VELOCITY_MPS,
    MAX_WORLD_XY_VELOCITY_MPS
  );
  const float desiredWorldZVelocity = clampf(
    runPid(
      flightCommand.zPos,
      posZPid,
      zError,
      dtSeconds,
      MAX_WORLD_Z_VELOCITY_MPS
    ),
    -MAX_WORLD_Z_VELOCITY_MPS,
    MAX_WORLD_Z_VELOCITY_MPS
  );

  const float worldXCommand = clampf(
    runPid(
      flightCommand.xyVel,
      xVelPid,
      desiredWorldXVelocity - worldXVelocity,
      dtSeconds,
      1.0f
    ),
    -1.0f,
    1.0f
  );
  const float worldYCommand = clampf(
    runPid(
      flightCommand.xyVel,
      yVelPid,
      desiredWorldYVelocity - worldYVelocity,
      dtSeconds,
      1.0f
    ),
    -1.0f,
    1.0f
  );
  const float zVelocityCommand = clampf(
    runPid(
      flightCommand.zVel,
      zVelPid,
      desiredWorldZVelocity - worldZVelocity,
      dtSeconds,
      1.0f
    ),
    -1.0f,
    1.0f
  );
  const float desiredYawRate = clampf(
    runPid(
      flightCommand.yawPos,
      yawOuterPid,
      yawError,
      dtSeconds,
      45.0f
    ),
    -flightCommand.maxYawRateDeg,
    flightCommand.maxYawRateDeg
  );

  const float yawRad = flightCommand.rotation[0] * DEG_TO_RAD_F;
  // Low-Cost-Mocap drives a separate flight controller over SBUS. Here we
  // adapt the same position -> velocity cascade into normalized world-frame
  // demands, then turn those into tilt/throttle targets for our direct mixer.
  const float forwardCommand = clampf(
    (cosf(yawRad) * worldXCommand) + (sinf(yawRad) * worldYCommand),
    -1.0f,
    1.0f
  );
  const float rightCommand = clampf(
    (sinf(yawRad) * worldXCommand) - (cosf(yawRad) * worldYCommand),
    -1.0f,
    1.0f
  );

  const float desiredPitchDeg = -forwardCommand * flightCommand.maxTiltDeg;
  const float desiredRollDeg = rightCommand * flightCommand.maxTiltDeg;
  const float requestedThrottle = throttleFromNormalizedCommand(zVelocityCommand);
  const float armBlend = clampf(
    static_cast<float>(millis() - armStartMs) / static_cast<float>(ARM_RAMP_MS),
    0.0f,
    1.0f
  );
  const float throttleBase = flightCommand.minThrottle
    + ((requestedThrottle - flightCommand.minThrottle) * armBlend);

  const float rollCorrection = runPid(
    flightCommand.roll,
    rollPid,
    desiredRollDeg - imuState.rollDeg,
    dtSeconds,
    20.0f
  );
  const float pitchCorrection = runPid(
    flightCommand.pitch,
    pitchPid,
    desiredPitchDeg - imuState.pitchDeg,
    dtSeconds,
    20.0f
  );
  const float yawCorrection = runPid(
    flightCommand.yawRate,
    yawRatePid,
    desiredYawRate - imuState.gyroZDegPerSec,
    dtSeconds,
    60.0f
  );

  float mixedOutputs[MOTOR_COUNT] = {};
  for (uint8_t index = 0; index < MOTOR_COUNT; ++index) {
    mixedOutputs[index] = throttleBase
      + (rollCorrection * MOTOR_ROLL_SIGN[index])
      + (pitchCorrection * MOTOR_PITCH_SIGN[index])
      + (yawCorrection * MOTOR_YAW_SIGN[index]);
  }

  normalizeMotorOutputs(mixedOutputs);
  writeMotorOutputs(mixedOutputs);
}

void processIncomingPayloads() {
  char payloadBuffer[MAX_PAYLOAD_LENGTH] = {};
  if (!pullPendingPayload(payloadBuffer, sizeof(payloadBuffer))) {
    return;
  }

  FlightCommand nextCommand = flightCommand;
  if (!parseFlightCommandPayload(payloadBuffer, nextCommand)) {
    Serial.print("Failed to parse command payload: ");
    Serial.println(payloadBuffer);
    return;
  }

  flightCommand = nextCommand;
  lastCommandRxMs = millis();
}

void printStatus() {
  if ((millis() - lastStatusPrintMs) < STATUS_PRINT_INTERVAL_MS) {
    return;
  }
  lastStatusPrintMs = millis();

  const bool commandFresh = (millis() - lastCommandRxMs) <= COMMAND_TIMEOUT_MS;
  Serial.printf(
    "fresh=%d armed=%d ok=%d imu=(%.1f,%.1f,%.1f) mocap=(%.2f,%.2f,%.2f | %.1f) vel=(%.2f,%.2f,%.2f) target=(%.2f,%.2f,%.2f | %.1f) motors=(%.2f,%.2f,%.2f,%.2f)\n",
    commandFresh ? 1 : 0,
    flightCommand.armed ? 1 : 0,
    flightCommand.spatialValid ? 1 : 0,
    imuState.rollDeg,
    imuState.pitchDeg,
    imuState.gyroZDegPerSec,
    flightCommand.position[0],
    flightCommand.position[1],
    flightCommand.position[2],
    flightCommand.rotation[0],
    flightCommand.velocity[0],
    flightCommand.velocity[1],
    flightCommand.velocity[2],
    flightCommand.target[0],
    flightCommand.target[1],
    flightCommand.target[2],
    flightCommand.target[3],
    motorOutputs[0],
    motorOutputs[1],
    motorOutputs[2],
    motorOutputs[3]
  );
}

void setup() {
  flightCommand = makeDefaultFlightCommand();
  resetImuFilters();

  Serial.begin(USB_BAUD_RATE);
  const unsigned long waitStart = millis();
  while (!Serial && (millis() - waitStart) < SERIAL_WAIT_MS) {
    delay(10);
  }

  delay(250);
  Serial.println();
  Serial.println("ESP32-S2 hover controller starting...");
  WiFi.mode(WIFI_STA);
  Serial.print("ESP32-S2 MAC: ");
  Serial.println(WiFi.macAddress());

  setupMotors();

  if (!initEspNowReceiver()) {
    Serial.println("ESP-NOW receiver failed to initialise.");
  }

  bringUpMpu6050();

  Serial.println("Hover controller ready. Waiting for compact control payloads.");
}

void loop() {
  static unsigned long lastLoopMicros = micros();
  const unsigned long nowMicros = micros();
  float dtSeconds = static_cast<float>(nowMicros - lastLoopMicros) / 1000000.0f;
  lastLoopMicros = nowMicros;
  dtSeconds = clampf(dtSeconds, 0.001f, 0.02f);

  processIncomingPayloads();

  if (!ensureMpu6050Ready()) {
    stopMotors();
  } else if (!updateImuEstimate(dtSeconds)) {
    stopMotors();
  } else {
    processLevelCalibrationRequest();
    runHoverController(dtSeconds);
  }

  printStatus();
  delay(2);
}
