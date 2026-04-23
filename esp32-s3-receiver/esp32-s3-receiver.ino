#include <WiFi.h>
#include <esp_now.h>
#include <math.h>
#include <cstring>
#include <cstdlib>

constexpr unsigned long USB_BAUD_RATE = 115200;
constexpr unsigned long FC_BAUD_RATE = 420000;
constexpr unsigned long SERIAL_WAIT_MS = 2000;
constexpr unsigned long STATUS_PRINT_INTERVAL_MS = 1000;
constexpr unsigned long COMMAND_TIMEOUT_MS = 250;
constexpr unsigned long FC_ATTITUDE_TIMEOUT_MS = 500;
// Keep manual USB testing human-friendly without changing the outer-loop timeout.
constexpr unsigned long MANUAL_COMMAND_TIMEOUT_MS = 5000;
// When manual arming starts, briefly keep throttle low so Betaflight can arm.
constexpr unsigned long MANUAL_ARM_THROTTLE_HOLD_MS = 1200;
constexpr uint16_t MANUAL_ARM_LOW_THROTTLE_RC = 1000;
constexpr size_t MAX_PAYLOAD_LENGTH = 251;
constexpr size_t MAX_SERIAL_FRAME_LENGTH = MAX_PAYLOAD_LENGTH - 1;
constexpr size_t CRSF_MAX_FRAME_LENGTH = 64;
constexpr uint8_t CRSF_FRAME_TYPE_ATTITUDE = 0x1E;

constexpr uint8_t UART2_RX_PIN = 7;
constexpr uint8_t UART2_TX_PIN = 6;

constexpr float DEG_TO_RAD_F = 0.0174532925f;
constexpr float RAD_TO_DEG_F = 57.2957795f;
constexpr float CRSF_ATTITUDE_LSB_RAD = 0.0001f;
constexpr float MAX_WORLD_XY_VELOCITY_MPS = 0.8f;
constexpr float MAX_WORLD_Z_VELOCITY_MPS = 0.5f;
// Match these to the flight-controller's configured angle/rate limits.
constexpr float FC_FULL_SCALE_TILT_DEG = 45.0f;
constexpr float FC_FULL_SCALE_YAW_RATE_DEG = 360.0f;

// Set these to -1.0f if your flight-controller channel directions are inverted.
constexpr float RC_ROLL_SIGN = 1.0f;
constexpr float RC_PITCH_SIGN = 1.0f;
constexpr float RC_YAW_SIGN = 1.0f;

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
  float velocity[3];
  float rotation[3];
  float target[4];
  PidGains xyPos;
  PidGains zPos;
  PidGains yawPos;
  PidGains xyVel;
  PidGains zVel;
  float hoverThrottle;
  float minThrottle;
  float maxThrottle;
  float maxTiltDeg;
  float maxYawRateDeg;
};

struct ManualControlCommand {
  bool enabled;
  bool armed;
  bool useRawRc;
  float roll;
  float pitch;
  float throttle;
  float yaw;
  uint16_t rawRoll;
  uint16_t rawPitch;
  uint16_t rawThrottle;
  uint16_t rawYaw;
};

portMUX_TYPE payloadMux = portMUX_INITIALIZER_UNLOCKED;
char pendingPayload[MAX_PAYLOAD_LENGTH] = {};
volatile bool pendingPayloadReady = false;

FlightCommand flightCommand = {};
ManualControlCommand manualControl = {};

char incomingSerialFrame[MAX_SERIAL_FRAME_LENGTH + 1] = {};
size_t incomingSerialFrameLength = 0;
bool incomingSerialFrameOverflow = false;

PidState posXPid = {};
PidState posYPid = {};
PidState posZPid = {};
PidState yawOuterPid = {};
PidState xVelPid = {};
PidState yVelPid = {};
PidState zVelPid = {};

unsigned long lastCommandRxMs = 0;
unsigned long lastManualCommandRxMs = 0;
unsigned long lastStatusPrintMs = 0;
unsigned long manualArmThrottleHoldStartedMs = 0;
bool manualArmCommandLatched = false;
bool manualArmThrottleHoldActive = false;

uint16_t lastRcRoll = 1500;
uint16_t lastRcPitch = 1500;
uint16_t lastRcThrottle = 1000;
uint16_t lastRcYaw = 1500;
uint16_t lastRcArm = 1000;

float lastDesiredWorldXVelocity = 0.0f;
float lastDesiredWorldYVelocity = 0.0f;
float lastDesiredWorldZVelocity = 0.0f;
float lastDesiredYawRate = 0.0f;
float lastDesiredRollDeg = 0.0f;
float lastDesiredPitchDeg = 0.0f;
float lastFcPitchDeg = 0.0f;
float lastFcRollDeg = 0.0f;
float lastFcYawDeg = 0.0f;
unsigned long lastFcAttitudeRxMs = 0;

uint8_t incomingCrsfFrame[CRSF_MAX_FRAME_LENGTH] = {};
size_t incomingCrsfFrameLength = 0;
size_t incomingCrsfFrameExpectedLength = 0;

void resetManualArmAssist() {
  manualArmCommandLatched = false;
  manualArmThrottleHoldActive = false;
  manualArmThrottleHoldStartedMs = 0;
}

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

int16_t readInt16BigEndian(const uint8_t *data) {
  return static_cast<int16_t>(
    (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1])
  );
}

PidGains makePidGains(float kp, float ki, float kd) {
  PidGains gains = {kp, ki, kd};
  return gains;
}

ManualControlCommand makeDefaultManualControl() {
  ManualControlCommand command = {};
  command.enabled = false;
  command.armed = false;
  command.useRawRc = false;
  command.roll = 0.0f;
  command.pitch = 0.0f;
  command.throttle = 0.0f;
  command.yaw = 0.0f;
  command.rawRoll = 1500;
  command.rawPitch = 1500;
  command.rawThrottle = 1000;
  command.rawYaw = 1500;
  return command;
}

FlightCommand makeDefaultFlightCommand() {
  FlightCommand command = {};
  command.armed = false;
  command.spatialValid = false;
  command.position[0] = 0.0f;
  command.position[1] = 0.0f;
  command.position[2] = 0.0f;
  command.velocity[0] = 0.0f;
  command.velocity[1] = 0.0f;
  command.velocity[2] = 0.0f;
  command.rotation[0] = 0.0f;
  command.rotation[1] = 0.0f;
  command.rotation[2] = 0.0f;
  command.target[0] = 0.0f;
  command.target[1] = 0.0f;
  command.target[2] = 0.25f;
  command.target[3] = 0.0f;
  command.xyPos = makePidGains(1.0f, 0.0f, 0.0f);
  command.zPos = makePidGains(1.5f, 0.0f, 0.0f);
  command.yawPos = makePidGains(0.3f, 0.1f, 0.05f);
  command.xyVel = makePidGains(0.2f, 0.03f, 0.05f);
  command.zVel = makePidGains(0.3f, 0.1f, 0.05f);
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
  resetPidState(xVelPid);
  resetPidState(yVelPid);
  resetPidState(zVelPid);
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

uint16_t normalizedAxisToRc(float normalizedCommand, float sign = 1.0f) {
  const float signedCommand = clampf(normalizedCommand * sign, -1.0f, 1.0f);
  return static_cast<uint16_t>(1500.0f + (signedCommand * 500.0f));
}

uint16_t throttleToRc(float normalizedThrottle) {
  const float throttle = clampf(normalizedThrottle, 0.0f, 1.0f);
  return static_cast<uint16_t>(1000.0f + (throttle * 1000.0f));
}

float rcAxisToNormalized(uint16_t rcValue) {
  return clampf((static_cast<float>(rcValue) - 1500.0f) / 500.0f, -1.0f, 1.0f);
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

bool parseFloatValue(const char *json, const char *key, float &value) {
  const char *cursor = strstr(json, key);
  if (cursor == nullptr) {
    return false;
  }

  cursor += strlen(key);
  char *parseEnd = nullptr;
  value = strtof(cursor, &parseEnd);
  return parseEnd != cursor;
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

bool parsePidProfilePayload(const char *payload, FlightCommand &parsed) {
  float pidBundle[24] = {};
  float values3[3] = {};

  if (parseFloatArray(payload, "\"u\":", pidBundle, 24)) {
    parsed.xyPos = makePidGains(pidBundle[0], pidBundle[1], pidBundle[2]);
    parsed.zPos = makePidGains(pidBundle[3], pidBundle[4], pidBundle[5]);
    parsed.yawPos = makePidGains(pidBundle[6], pidBundle[7], pidBundle[8]);
    parsed.xyVel = makePidGains(pidBundle[9], pidBundle[10], pidBundle[11]);
    parsed.zVel = makePidGains(pidBundle[12], pidBundle[13], pidBundle[14]);
    return true;
  }

  if (!parseFloatArray(payload, "\"xp\":", values3, 3)) {
    return false;
  }
  parsed.xyPos = makePidGains(values3[0], values3[1], values3[2]);

  if (!parseFloatArray(payload, "\"zp\":", values3, 3)) {
    return false;
  }
  parsed.zPos = makePidGains(values3[0], values3[1], values3[2]);

  if (!parseFloatArray(payload, "\"yp\":", values3, 3)) {
    return false;
  }
  parsed.yawPos = makePidGains(values3[0], values3[1], values3[2]);

  if (!parseFloatArray(payload, "\"xv\":", values3, 3)) {
    return false;
  }
  parsed.xyVel = makePidGains(values3[0], values3[1], values3[2]);

  if (!parseFloatArray(payload, "\"zv\":", values3, 3)) {
    return false;
  }
  parsed.zVel = makePidGains(values3[0], values3[1], values3[2]);

  return true;
}

bool parseFlightCommandPayload(const char *payload, FlightCommand &nextCommand) {
  FlightCommand parsed = flightCommand;
  float targetValues[4] = {};
  float limits[5] = {};

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
  if (!parseFloatArrayEither(payload, "\"g\":", "\"tg\":", targetValues, 4)) {
    return false;
  }
  memcpy(parsed.target, targetValues, sizeof(targetValues));

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

  nextCommand = parsed;
  return true;
}

bool parseManualControlPayload(const char *payload, ManualControlCommand &nextCommand) {
  ManualControlCommand parsed = manualControl;
  float axes[4] = {parsed.roll, parsed.pitch, parsed.throttle, parsed.yaw};
  float rawAxes[4] = {
    static_cast<float>(parsed.rawRoll),
    static_cast<float>(parsed.rawPitch),
    static_cast<float>(parsed.rawThrottle),
    static_cast<float>(parsed.rawYaw),
  };

  bool manualEnabled = parsed.enabled;
  const bool hasManualFlag = parseBoolFlagEither(payload, "\"manual\":", "\"man\":", manualEnabled);
  const bool hasAxesArray = parseFloatArrayEither(payload, "\"c\":", "\"axes\":", axes, 4);
  const bool hasRawArray = parseFloatArrayEither(payload, "\"rc\":", "\"raw\":", rawAxes, 4);
  const bool hasRoll = parseFloatValue(payload, "\"roll\":", axes[0]);
  const bool hasPitch = parseFloatValue(payload, "\"pitch\":", axes[1]);
  const bool hasThrottle = parseFloatValue(payload, "\"throttle\":", axes[2]);
  const bool hasYaw = parseFloatValue(payload, "\"yaw\":", axes[3]);
  const bool hasNamedAxes = hasRoll || hasPitch || hasThrottle || hasYaw;
  const bool hasArmFlag = parseBoolFlagEither(payload, "\"a\":", "\"arm\":", parsed.armed);
  const bool hasControlValues = hasAxesArray || hasRawArray || hasNamedAxes;

  if (!(hasManualFlag || hasControlValues || (parsed.enabled && hasArmFlag))) {
    return false;
  }

  if (!hasManualFlag && (hasControlValues || (parsed.enabled && hasArmFlag))) {
    manualEnabled = true;
  }

  parsed.enabled = manualEnabled;
  if (!parsed.enabled) {
    parsed.armed = false;
    parsed.useRawRc = false;
    nextCommand = parsed;
    return true;
  }

  if (hasAxesArray || hasNamedAxes) {
    parsed.roll = clampf(axes[0], -1.0f, 1.0f);
    parsed.pitch = clampf(axes[1], -1.0f, 1.0f);
    parsed.throttle = clampf(axes[2], 0.0f, 1.0f);
    parsed.yaw = clampf(axes[3], -1.0f, 1.0f);
    parsed.useRawRc = false;
  }

  if (hasRawArray) {
    parsed.rawRoll = static_cast<uint16_t>(clampf(rawAxes[0], 1000.0f, 2000.0f));
    parsed.rawPitch = static_cast<uint16_t>(clampf(rawAxes[1], 1000.0f, 2000.0f));
    parsed.rawThrottle = static_cast<uint16_t>(clampf(rawAxes[2], 1000.0f, 2000.0f));
    parsed.rawYaw = static_cast<uint16_t>(clampf(rawAxes[3], 1000.0f, 2000.0f));
    parsed.useRawRc = true;
  }

  nextCommand = parsed;
  return true;
}

void onDataReceived(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int dataLen) {
  (void)recvInfo;

  if (dataLen <= 0) {
    return;
  }

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

uint8_t crsf_crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0xD5;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

void handleFlightControllerCrsfFrame(const uint8_t *frame, size_t frameLength) {
  if (frameLength < 4) {
    return;
  }

  const uint8_t payloadLength = frame[1];
  if (payloadLength < 2 || frameLength != static_cast<size_t>(payloadLength) + 2) {
    return;
  }

  const uint8_t receivedCrc = frame[frameLength - 1];
  const uint8_t calculatedCrc = crsf_crc8(&frame[2], payloadLength - 1);
  if (receivedCrc != calculatedCrc) {
    return;
  }

  const uint8_t frameType = frame[2];
  const uint8_t *payload = &frame[3];
  const size_t payloadSize = payloadLength - 2;
  if (frameType != CRSF_FRAME_TYPE_ATTITUDE || payloadSize < 6) {
    return;
  }

  lastFcPitchDeg = wrapDegrees(
    static_cast<float>(readInt16BigEndian(payload)) * CRSF_ATTITUDE_LSB_RAD * RAD_TO_DEG_F
  );
  lastFcRollDeg = wrapDegrees(
    static_cast<float>(readInt16BigEndian(payload + 2)) * CRSF_ATTITUDE_LSB_RAD * RAD_TO_DEG_F
  );
  lastFcYawDeg = wrapDegrees(
    static_cast<float>(readInt16BigEndian(payload + 4)) * CRSF_ATTITUDE_LSB_RAD * RAD_TO_DEG_F
  );
  lastFcAttitudeRxMs = millis();
}

void processFlightControllerTelemetry() {
  while (Serial2.available() > 0) {
    const uint8_t incomingByte = static_cast<uint8_t>(Serial2.read());

    if (incomingCrsfFrameLength == 0) {
      incomingCrsfFrame[incomingCrsfFrameLength++] = incomingByte;
      continue;
    }

    if (incomingCrsfFrameLength == 1) {
      if (incomingByte < 2 || incomingByte > (CRSF_MAX_FRAME_LENGTH - 2)) {
        incomingCrsfFrame[0] = incomingByte;
        incomingCrsfFrameLength = 1;
        incomingCrsfFrameExpectedLength = 0;
        continue;
      }

      incomingCrsfFrame[incomingCrsfFrameLength++] = incomingByte;
      incomingCrsfFrameExpectedLength = static_cast<size_t>(incomingByte) + 2;
      continue;
    }

    if (incomingCrsfFrameLength >= CRSF_MAX_FRAME_LENGTH) {
      incomingCrsfFrameLength = 0;
      incomingCrsfFrameExpectedLength = 0;
      continue;
    }

    incomingCrsfFrame[incomingCrsfFrameLength++] = incomingByte;
    if (
      incomingCrsfFrameExpectedLength > 0
      && incomingCrsfFrameLength >= incomingCrsfFrameExpectedLength
    ) {
      handleFlightControllerCrsfFrame(incomingCrsfFrame, incomingCrsfFrameLength);
      incomingCrsfFrameLength = 0;
      incomingCrsfFrameExpectedLength = 0;
    }
  }
}

void sendCRSF(uint16_t chRoll, uint16_t chPitch, uint16_t chThrottle, uint16_t chYaw, uint16_t chArm) {
  uint16_t channels[16] = {
    1500, 1500, 1000, 1500, 1000, 1000, 1000, 1000,
    1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000
  };
  channels[0] = chRoll;
  channels[1] = chPitch;
  channels[2] = chThrottle;
  channels[3] = chYaw;
  channels[4] = chArm;

  uint8_t packet[26];
  packet[0] = 0xC8;
  packet[1] = 24;
  packet[2] = 0x16;

  uint8_t payload[22] = {};
  uint32_t bits = 0;
  uint8_t bitsAvailable = 0;
  uint8_t byteIndex = 0;

  for (int channel = 0; channel < 16; ++channel) {
    const uint32_t crsfValue = map(channels[channel], 1000, 2000, 172, 1811);
    bits |= (crsfValue << bitsAvailable);
    bitsAvailable += 11;

    while (bitsAvailable >= 8) {
      payload[byteIndex++] = bits & 0xFF;
      bits >>= 8;
      bitsAvailable -= 8;
    }
  }

  for (int i = 0; i < 22; ++i) {
    packet[3 + i] = payload[i];
  }

  packet[25] = crsf_crc8(&packet[2], 23);
  Serial2.write(packet, sizeof(packet));
}

void sendSafeDisarmedFrame() {
  lastRcRoll = 1500;
  lastRcPitch = 1500;
  lastRcThrottle = 1000;
  lastRcYaw = 1500;
  lastRcArm = 1000;
  lastDesiredWorldXVelocity = 0.0f;
  lastDesiredWorldYVelocity = 0.0f;
  lastDesiredWorldZVelocity = 0.0f;
  lastDesiredYawRate = 0.0f;
  lastDesiredRollDeg = 0.0f;
  lastDesiredPitchDeg = 0.0f;
  sendCRSF(lastRcRoll, lastRcPitch, lastRcThrottle, lastRcYaw, lastRcArm);
}

bool applyFlightCommandPayload(const char *payload, const char *sourceLabel) {
  FlightCommand nextCommand = flightCommand;
  if (!parseFlightCommandPayload(payload, nextCommand)) {
    Serial.print("Failed to parse ");
    Serial.print(sourceLabel);
    Serial.print(" payload: ");
    Serial.println(payload);
    return false;
  }

  flightCommand = nextCommand;
  lastCommandRxMs = millis();
  return true;
}

bool applyUsbSerialPayload(const char *payload) {
  ManualControlCommand nextManual = manualControl;
  if (parseManualControlPayload(payload, nextManual)) {
    manualControl = nextManual;
    lastManualCommandRxMs = millis();
    return true;
  }

  return applyFlightCommandPayload(payload, "USB serial");
}

void processIncomingSerialFrame() {
  if (incomingSerialFrameOverflow || incomingSerialFrameLength == 0) {
    if (incomingSerialFrameOverflow) {
      Serial.println("Dropped oversized USB serial payload.");
    }
    incomingSerialFrameLength = 0;
    incomingSerialFrameOverflow = false;
    return;
  }

  incomingSerialFrame[incomingSerialFrameLength] = '\0';
  applyUsbSerialPayload(incomingSerialFrame);
  incomingSerialFrameLength = 0;
  incomingSerialFrameOverflow = false;
}

void processIncomingSerial() {
  while (Serial.available() > 0) {
    const char incomingByte = static_cast<char>(Serial.read());
    if (incomingByte == '\r') {
      continue;
    }
    if (incomingByte == '\n') {
      processIncomingSerialFrame();
      continue;
    }

    if (incomingSerialFrameOverflow) {
      continue;
    }
    if (incomingSerialFrameLength >= MAX_SERIAL_FRAME_LENGTH) {
      incomingSerialFrameOverflow = true;
      continue;
    }

    incomingSerialFrame[incomingSerialFrameLength++] = incomingByte;
  }
}

void processIncomingEspNowPayloads() {
  char payloadBuffer[MAX_PAYLOAD_LENGTH] = {};
  if (!pullPendingPayload(payloadBuffer, sizeof(payloadBuffer))) {
    return;
  }

  applyFlightCommandPayload(payloadBuffer, "ESP-NOW");
}

bool runManualControlToFlightController() {
  if (!manualControl.enabled) {
    resetManualArmAssist();
    return false;
  }

  resetAllPidStates();

  const bool manualFresh = (millis() - lastManualCommandRxMs) <= MANUAL_COMMAND_TIMEOUT_MS;
  if (!manualFresh || !manualControl.armed) {
    resetManualArmAssist();
    sendSafeDisarmedFrame();
    return true;
  }

  if (!manualArmCommandLatched) {
    manualArmCommandLatched = true;
    manualArmThrottleHoldActive = true;
    manualArmThrottleHoldStartedMs = millis();
  }
  if (
    manualArmThrottleHoldActive
    && (millis() - manualArmThrottleHoldStartedMs) >= MANUAL_ARM_THROTTLE_HOLD_MS
  ) {
    manualArmThrottleHoldActive = false;
  }

  float rollNormalized = manualControl.roll;
  float pitchNormalized = manualControl.pitch;
  float yawNormalized = manualControl.yaw;
  uint16_t desiredThrottleRc = throttleToRc(manualControl.throttle);

  if (manualControl.useRawRc) {
    lastRcRoll = manualControl.rawRoll;
    lastRcPitch = manualControl.rawPitch;
    desiredThrottleRc = manualControl.rawThrottle;
    lastRcYaw = manualControl.rawYaw;
    rollNormalized = rcAxisToNormalized(lastRcRoll);
    pitchNormalized = rcAxisToNormalized(lastRcPitch);
    yawNormalized = rcAxisToNormalized(lastRcYaw);
  } else {
    lastRcRoll = normalizedAxisToRc(rollNormalized, RC_ROLL_SIGN);
    lastRcPitch = normalizedAxisToRc(pitchNormalized, RC_PITCH_SIGN);
    lastRcYaw = normalizedAxisToRc(yawNormalized, RC_YAW_SIGN);
  }

  lastRcThrottle = manualArmThrottleHoldActive ? MANUAL_ARM_LOW_THROTTLE_RC : desiredThrottleRc;
  lastRcArm = 2000;
  lastDesiredWorldXVelocity = 0.0f;
  lastDesiredWorldYVelocity = 0.0f;
  lastDesiredWorldZVelocity = 0.0f;
  lastDesiredRollDeg = rollNormalized * FC_FULL_SCALE_TILT_DEG;
  lastDesiredPitchDeg = pitchNormalized * FC_FULL_SCALE_TILT_DEG;
  lastDesiredYawRate = yawNormalized * FC_FULL_SCALE_YAW_RATE_DEG;
  sendCRSF(lastRcRoll, lastRcPitch, lastRcThrottle, lastRcYaw, lastRcArm);
  return true;
}

void runOuterLoopToFlightController(float dtSeconds) {
  if (runManualControlToFlightController()) {
    return;
  }

  const bool commandFresh = (millis() - lastCommandRxMs) <= COMMAND_TIMEOUT_MS;
  const bool controlReady = commandFresh && flightCommand.armed && flightCommand.spatialValid;

  if (!controlReady) {
    resetAllPidStates();
    sendSafeDisarmedFrame();
    return;
  }

  const float xError = flightCommand.target[0] - flightCommand.position[0];
  const float yError = flightCommand.target[1] - flightCommand.position[1];
  const float zError = flightCommand.target[2] - flightCommand.position[2];
  const float yawError = wrapDegrees(flightCommand.target[3] - flightCommand.rotation[0]);

  lastDesiredWorldXVelocity = clampf(
    runPid(flightCommand.xyPos, posXPid, xError, dtSeconds, MAX_WORLD_XY_VELOCITY_MPS),
    -MAX_WORLD_XY_VELOCITY_MPS,
    MAX_WORLD_XY_VELOCITY_MPS
  );
  lastDesiredWorldYVelocity = clampf(
    runPid(flightCommand.xyPos, posYPid, yError, dtSeconds, MAX_WORLD_XY_VELOCITY_MPS),
    -MAX_WORLD_XY_VELOCITY_MPS,
    MAX_WORLD_XY_VELOCITY_MPS
  );
  lastDesiredWorldZVelocity = clampf(
    runPid(flightCommand.zPos, posZPid, zError, dtSeconds, MAX_WORLD_Z_VELOCITY_MPS),
    -MAX_WORLD_Z_VELOCITY_MPS,
    MAX_WORLD_Z_VELOCITY_MPS
  );

  const float worldXCommand = clampf(
    runPid(
      flightCommand.xyVel,
      xVelPid,
      lastDesiredWorldXVelocity - flightCommand.velocity[0],
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
      lastDesiredWorldYVelocity - flightCommand.velocity[1],
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
      lastDesiredWorldZVelocity - flightCommand.velocity[2],
      dtSeconds,
      1.0f
    ),
    -1.0f,
    1.0f
  );
  lastDesiredYawRate = clampf(
    runPid(flightCommand.yawPos, yawOuterPid, yawError, dtSeconds, 45.0f),
    -flightCommand.maxYawRateDeg,
    flightCommand.maxYawRateDeg
  );

  const float yawRad = flightCommand.rotation[0] * DEG_TO_RAD_F;
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

  const float requestedThrottle = throttleFromNormalizedCommand(zVelocityCommand);
  lastDesiredRollDeg = rightCommand * flightCommand.maxTiltDeg;
  lastDesiredPitchDeg = -forwardCommand * flightCommand.maxTiltDeg;
  const float rollNormalized = lastDesiredRollDeg / FC_FULL_SCALE_TILT_DEG;
  const float pitchNormalized = lastDesiredPitchDeg / FC_FULL_SCALE_TILT_DEG;
  const float yawNormalized = lastDesiredYawRate / FC_FULL_SCALE_YAW_RATE_DEG;

  lastRcRoll = normalizedAxisToRc(rollNormalized, RC_ROLL_SIGN);
  lastRcPitch = normalizedAxisToRc(pitchNormalized, RC_PITCH_SIGN);
  lastRcThrottle = throttleToRc(requestedThrottle);
  lastRcYaw = normalizedAxisToRc(yawNormalized, RC_YAW_SIGN);
  lastRcArm = 2000;

  sendCRSF(lastRcRoll, lastRcPitch, lastRcThrottle, lastRcYaw, lastRcArm);
}

void printStatus() {
  if ((millis() - lastStatusPrintMs) < STATUS_PRINT_INTERVAL_MS) {
    return;
  }
  lastStatusPrintMs = millis();

  const bool autoFresh = (millis() - lastCommandRxMs) <= COMMAND_TIMEOUT_MS;
  const bool manualFresh = manualControl.enabled && ((millis() - lastManualCommandRxMs) <= MANUAL_COMMAND_TIMEOUT_MS);
  const bool fcAttitudeFresh = (millis() - lastFcAttitudeRxMs) <= FC_ATTITUDE_TIMEOUT_MS;
  const char *mode = "safe";
  if (manualControl.enabled) {
    if (!manualFresh) {
      mode = "manual_timeout";
    } else if (!manualControl.armed) {
      mode = "manual_disarmed";
    } else if (manualArmThrottleHoldActive) {
      mode = "manual_arm_hold";
    } else {
      mode = "manual";
    }
  } else if (autoFresh && flightCommand.armed && flightCommand.spatialValid) {
    mode = "outer";
  } else if (autoFresh && !flightCommand.armed) {
    mode = "auto_disarmed";
  } else if (autoFresh && !flightCommand.spatialValid) {
    mode = "waiting_pose";
  }

  Serial.printf(
    "mode=%s autoFresh=%d autoArm=%d ok=%d manEn=%d manFresh=%d manArm=%d manRaw=%d fcAttOk=%d fcAtt=(p=%.1f,r=%.1f,y=%.1f) pos=(%.2f,%.2f,%.2f) vel=(%.2f,%.2f,%.2f) yaw=%.1f target=(%.2f,%.2f,%.2f|%.1f) outerVel=(%.2f,%.2f,%.2f) tilt=(%.1f,%.1f) yawRate=%.1f rc=(%u,%u,%u,%u,%u)\n",
    mode,
    autoFresh ? 1 : 0,
    flightCommand.armed ? 1 : 0,
    flightCommand.spatialValid ? 1 : 0,
    manualControl.enabled ? 1 : 0,
    manualFresh ? 1 : 0,
    manualControl.armed ? 1 : 0,
    manualControl.useRawRc ? 1 : 0,
    fcAttitudeFresh ? 1 : 0,
    lastFcPitchDeg,
    lastFcRollDeg,
    lastFcYawDeg,
    flightCommand.position[0],
    flightCommand.position[1],
    flightCommand.position[2],
    flightCommand.velocity[0],
    flightCommand.velocity[1],
    flightCommand.velocity[2],
    flightCommand.rotation[0],
    flightCommand.target[0],
    flightCommand.target[1],
    flightCommand.target[2],
    flightCommand.target[3],
    lastDesiredWorldXVelocity,
    lastDesiredWorldYVelocity,
    lastDesiredWorldZVelocity,
    lastDesiredRollDeg,
    lastDesiredPitchDeg,
    lastDesiredYawRate,
    static_cast<unsigned>(lastRcRoll),
    static_cast<unsigned>(lastRcPitch),
    static_cast<unsigned>(lastRcThrottle),
    static_cast<unsigned>(lastRcYaw),
    static_cast<unsigned>(lastRcArm)
  );
}

void setup() {
  flightCommand = makeDefaultFlightCommand();
  manualControl = makeDefaultManualControl();
  resetAllPidStates();

  Serial.begin(USB_BAUD_RATE);
  const unsigned long waitStart = millis();
  while (!Serial && (millis() - waitStart) < SERIAL_WAIT_MS) {
    delay(10);
  }

  delay(250);
  Serial.println();
  Serial.println("ESP32-S3 mocap receiver starting...");
  WiFi.mode(WIFI_STA);
  Serial.print("ESP32-S3 MAC: ");
  Serial.println(WiFi.macAddress());

  Serial2.begin(FC_BAUD_RATE, SERIAL_8N1, UART2_RX_PIN, UART2_TX_PIN);
  sendSafeDisarmedFrame();

  if (!initEspNowReceiver()) {
    Serial.println("ESP-NOW receiver failed to initialise.");
  }

  Serial.println("Receiver ready. Waiting for ESP-NOW mocap payloads or USB serial manual commands.");
  Serial.println("Manual USB example: {\"manual\":1,\"arm\":1,\"axes\":[0,0,0,0]}");
  Serial.printf("Manual USB timeout: %lu ms\n", static_cast<unsigned long>(MANUAL_COMMAND_TIMEOUT_MS));
  Serial.printf("Manual arm hold: %lu ms at RC throttle %u\n",
    static_cast<unsigned long>(MANUAL_ARM_THROTTLE_HOLD_MS),
    static_cast<unsigned>(MANUAL_ARM_LOW_THROTTLE_RC)
  );
}

void loop() {
  static unsigned long lastLoopMicros = micros();
  const unsigned long nowMicros = micros();
  float dtSeconds = static_cast<float>(nowMicros - lastLoopMicros) / 1000000.0f;
  lastLoopMicros = nowMicros;
  dtSeconds = clampf(dtSeconds, 0.001f, 0.02f);

  processIncomingSerial();
  processIncomingEspNowPayloads();
  processFlightControllerTelemetry();
  runOuterLoopToFlightController(dtSeconds);
  printStatus();

  delay(2);
}
