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

constexpr uint8_t I2C_SDA_PIN = 11;
constexpr uint8_t I2C_SCL_PIN = 10;
constexpr uint8_t MPU6050_ADDRESS = 0x68;
constexpr uint8_t MOTOR_COUNT = 4;
constexpr uint8_t MOTOR_PINS[MOTOR_COUNT] = {5, 6, 3, 4};
constexpr int PWM_FREQUENCY_HZ = 20000;
constexpr int PWM_RESOLUTION_BITS = 10;
constexpr int PWM_MAX_DUTY = (1 << PWM_RESOLUTION_BITS) - 1;

constexpr float ACCEL_SCALE_LSB_PER_G = 16384.0f;
constexpr float GYRO_SCALE_LSB_PER_DEG_PER_SEC = 65.5f;
constexpr float RAD_TO_DEG_F = 57.2957795f;
constexpr float DEG_TO_RAD_F = 0.0174532925f;
constexpr float COMPLEMENTARY_ALPHA = 0.98f;
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
constexpr float MOTOR_PITCH_SIGN[MOTOR_COUNT] = {-1.0f, 1.0f, 1.0f, -1.0f};
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

struct FlightCommand {
  bool armed;
  bool spatialValid;
  float position[3];
  float rotation[3];
  float target[4];
  PidGains posX;
  PidGains posY;
  PidGains posZ;
  PidGains yaw;
  PidGains roll;
  PidGains pitch;
  PidGains yawRate;
  float hoverThrottle;
  float minThrottle;
  float maxThrottle;
  float maxTiltDeg;
  float maxYawRateDeg;
};

struct ImuState {
  float rollDeg;
  float pitchDeg;
  float yawDeg;
  float gyroXDegPerSec;
  float gyroYDegPerSec;
  float gyroZDegPerSec;
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
PidState rollPid = {};
PidState pitchPid = {};
PidState yawRatePid = {};

float motorOutputs[MOTOR_COUNT] = {};
float gyroBiasX = 0.0f;
float gyroBiasY = 0.0f;
float gyroBiasZ = 0.0f;

unsigned long lastCommandRxMs = 0;
unsigned long armStartMs = 0;
unsigned long lastStatusPrintMs = 0;
bool motorsEnabled = false;

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

FlightCommand makeDefaultFlightCommand() {
  FlightCommand command = {};
  command.armed = false;
  command.spatialValid = false;
  command.target[0] = 0.0f;
  command.target[1] = 0.0f;
  command.target[2] = 0.35f;
  command.target[3] = 0.0f;
  command.posX = makePidGains(6.0f, 0.0f, 2.2f);
  command.posY = makePidGains(6.0f, 0.0f, 2.2f);
  command.posZ = makePidGains(0.9f, 0.35f, 0.18f);
  command.yaw = makePidGains(4.5f, 0.0f, 0.12f);
  command.roll = makePidGains(0.022f, 0.0f, 0.0014f);
  command.pitch = makePidGains(0.022f, 0.0f, 0.0014f);
  command.yawRate = makePidGains(0.006f, 0.0f, 0.0f);
  command.hoverThrottle = 0.36f;
  command.minThrottle = 0.18f;
  command.maxThrottle = 0.82f;
  command.maxTiltDeg = 12.0f;
  command.maxYawRateDeg = 120.0f;
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

bool parseFlightCommandPayload(const char *payload, FlightCommand &nextCommand) {
  FlightCommand parsed = flightCommand;
  float values3[3] = {};
  float values4[4] = {};
  float limits[5] = {};
  float pidBundle[21] = {};

  if (!parseBoolFlagEither(payload, "\"a\":", "\"arm\":", parsed.armed)) {
    return false;
  }
  if (!parseBoolFlagEither(payload, "\"o\":", "\"ok\":", parsed.spatialValid)) {
    return false;
  }
  if (!parseFloatArray(payload, "\"p\":", parsed.position, 3)) {
    return false;
  }
  if (!parseFloatArray(payload, "\"r\":", parsed.rotation, 3)) {
    return false;
  }
  if (!parseFloatArrayEither(payload, "\"g\":", "\"tg\":", values4, 4)) {
    return false;
  }
  memcpy(parsed.target, values4, sizeof(values4));

  if (parseFloatArray(payload, "\"u\":", pidBundle, 21)) {
    parsed.posX = makePidGains(pidBundle[0], pidBundle[1], pidBundle[2]);
    parsed.posY = makePidGains(pidBundle[3], pidBundle[4], pidBundle[5]);
    parsed.posZ = makePidGains(pidBundle[6], pidBundle[7], pidBundle[8]);
    parsed.yaw = makePidGains(pidBundle[9], pidBundle[10], pidBundle[11]);
    parsed.roll = makePidGains(pidBundle[12], pidBundle[13], pidBundle[14]);
    parsed.pitch = makePidGains(pidBundle[15], pidBundle[16], pidBundle[17]);
    parsed.yawRate = makePidGains(pidBundle[18], pidBundle[19], pidBundle[20]);
  } else {
    if (!parseFloatArray(payload, "\"x\":", values3, 3)) {
      return false;
    }
    parsed.posX = gainsFromTriplet(values3);

    if (!parseFloatArray(payload, "\"y\":", values3, 3)) {
      return false;
    }
    parsed.posY = gainsFromTriplet(values3);

    if (!parseFloatArray(payload, "\"z\":", values3, 3)) {
      return false;
    }
    parsed.posZ = gainsFromTriplet(values3);

    if (!parseFloatArray(payload, "\"w\":", values3, 3)) {
      return false;
    }
    parsed.yaw = gainsFromTriplet(values3);

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
  }

  if (!parseFloatArrayEither(payload, "\"m\":", "\"lm\":", limits, 5)) {
    return false;
  }

  parsed.minThrottle = clampf(limits[1], 0.0f, 1.0f);
  parsed.hoverThrottle = clampf(limits[0], parsed.minThrottle, 1.0f);
  parsed.maxThrottle = clampf(limits[2], parsed.hoverThrottle, 1.0f);
  parsed.maxTiltDeg = clampf(limits[3], 1.0f, 45.0f);
  parsed.maxYawRateDeg = clampf(limits[4], 1.0f, 360.0f);

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

bool writeMpuRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readMpuRegisters(uint8_t startRegister, uint8_t *buffer, size_t length) {
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(startRegister);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  const size_t bytesRead = Wire.requestFrom(MPU6050_ADDRESS, static_cast<uint8_t>(length), static_cast<uint8_t>(true));
  if (bytesRead != length) {
    return false;
  }

  for (size_t index = 0; index < length; ++index) {
    buffer[index] = Wire.read();
  }
  return true;
}

bool initMpu6050() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, 400000);
  delay(50);

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

  writeMpuRegister(0x1A, 0x03);
  writeMpuRegister(0x1B, 0x08);
  writeMpuRegister(0x1C, 0x00);

  Serial.print("MPU6050 ready. WHO_AM_I = 0x");
  Serial.println(whoAmI, HEX);
  return true;
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
    return false;
  }

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

  imuState.gyroXDegPerSec = -sensorGyroYDegPerSec;
  imuState.gyroYDegPerSec = -sensorGyroXDegPerSec;
  imuState.gyroZDegPerSec = sensorGyroZDegPerSec;

  const float rollAccDeg = atan2f(bodyRightAccel, bodyVerticalAccel) * RAD_TO_DEG_F;
  const float pitchAccDeg = atan2f(
    -bodyForwardAccel,
    sqrtf((bodyRightAccel * bodyRightAccel) + (bodyVerticalAccel * bodyVerticalAccel))
  ) * RAD_TO_DEG_F;

  if (!imuState.ready) {
    imuState.rollDeg = rollAccDeg;
    imuState.pitchDeg = pitchAccDeg;
    imuState.yawDeg = 0.0f;
    imuState.ready = true;
  } else {
    imuState.rollDeg = (COMPLEMENTARY_ALPHA * (imuState.rollDeg + (imuState.gyroXDegPerSec * dtSeconds)))
      + ((1.0f - COMPLEMENTARY_ALPHA) * rollAccDeg);
    imuState.pitchDeg = (COMPLEMENTARY_ALPHA * (imuState.pitchDeg + (imuState.gyroYDegPerSec * dtSeconds)))
      + ((1.0f - COMPLEMENTARY_ALPHA) * pitchAccDeg);
    imuState.yawDeg = wrapDegrees(imuState.yawDeg + (imuState.gyroZDegPerSec * dtSeconds));
  }

  return true;
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

  const float worldXCommand = runPid(flightCommand.posX, posXPid, xError, dtSeconds, 0.4f);
  const float worldYCommand = runPid(flightCommand.posY, posYPid, yError, dtSeconds, 0.4f);
  const float throttleCorrection = runPid(flightCommand.posZ, posZPid, zError, dtSeconds, 0.5f);
  const float desiredYawRate = clampf(
    runPid(flightCommand.yaw, yawOuterPid, yawError, dtSeconds, 45.0f),
    -flightCommand.maxYawRateDeg,
    flightCommand.maxYawRateDeg
  );

  const float yawRad = flightCommand.rotation[0] * DEG_TO_RAD_F;
  const float forwardCommand = (cosf(yawRad) * worldXCommand) + (sinf(yawRad) * worldYCommand);
  const float rightCommand = (-sinf(yawRad) * worldXCommand) + (cosf(yawRad) * worldYCommand);

  const float desiredPitchDeg = clampf(-forwardCommand, -flightCommand.maxTiltDeg, flightCommand.maxTiltDeg);
  const float desiredRollDeg = clampf(rightCommand, -flightCommand.maxTiltDeg, flightCommand.maxTiltDeg);

  const float requestedThrottle = clampf(
    flightCommand.hoverThrottle + throttleCorrection,
    flightCommand.minThrottle,
    flightCommand.maxThrottle
  );
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
    "fresh=%d armed=%d ok=%d imu=(%.1f,%.1f,%.1f) mocap=(%.2f,%.2f,%.2f | %.1f) target=(%.2f,%.2f,%.2f | %.1f) motors=(%.2f,%.2f,%.2f,%.2f)\n",
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

  if (!initMpu6050()) {
    Serial.println("MPU6050 failed to initialise.");
  } else if (!calibrateGyroBias()) {
    Serial.println("MPU6050 gyro bias calibration failed.");
  }

  Serial.println("Hover controller ready. Waiting for compact control payloads.");
}

void loop() {
  static unsigned long lastLoopMicros = micros();
  const unsigned long nowMicros = micros();
  float dtSeconds = static_cast<float>(nowMicros - lastLoopMicros) / 1000000.0f;
  lastLoopMicros = nowMicros;
  dtSeconds = clampf(dtSeconds, 0.001f, 0.02f);

  processIncomingPayloads();

  if (!updateImuEstimate(dtSeconds)) {
    stopMotors();
  } else {
    runHoverController(dtSeconds);
  }

  printStatus();
  delay(2);
}
