#include <WiFi.h>
#include <esp_now.h>
#include <cstring>
#include <cstdlib>

constexpr unsigned long USB_BAUD_RATE = 1000000;
constexpr size_t MAX_JSON_PAYLOAD_LENGTH = 250;
constexpr size_t MAX_SERIAL_FRAME_LENGTH = MAX_JSON_PAYLOAD_LENGTH + 1;
constexpr unsigned long SERIAL_WAIT_MS = 2000;
constexpr unsigned long BRIDGE_STATUS_INTERVAL_MS = 1000;

uint8_t DRONE_MAC_ADDRESS[] = {0x60, 0x55, 0xF9, 0xDA, 0x4E, 0xD4};

struct EspNowMessage {
  char payload[MAX_JSON_PAYLOAD_LENGTH + 1];
};

char incomingFrame[MAX_SERIAL_FRAME_LENGTH + 1] = {};
size_t incomingFrameLength = 0;
bool incomingFrameOverflow = false;
unsigned long lastBridgeStatusMs = 0;
unsigned long lastAttemptWindowMs = 0;
uint32_t lastAttemptWindowCount = 0;
uint32_t serialFramesReceived = 0;
uint32_t espNowTxAttempts = 0;
uint32_t espNowTxOk = 0;
uint32_t espNowTxFail = 0;
uint32_t oversizeDrops = 0;
uint32_t lastTxSeq = 0;
uint32_t lastTxLatencyUs = 0;
uint32_t lastHandledCounterResetSequence = 0;
unsigned long lastTxStartMicros = 0;

void onDataSent(const wifi_tx_info_t *txInfo, esp_now_send_status_t status) {
  (void)txInfo;
  lastTxLatencyUs = static_cast<uint32_t>(micros() - lastTxStartMicros);
  if (status == ESP_NOW_SEND_SUCCESS) {
    espNowTxOk += 1;
  } else {
    espNowTxFail += 1;
  }
}

void onDataReceived(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int dataLen) {
  (void)recvInfo;
  if (data == nullptr || dataLen <= 0) {
    return;
  }

  Serial.write(data, static_cast<size_t>(dataLen));
  if (data[dataLen - 1] != '\n') {
    Serial.write('\n');
  }
}

bool tryExtractSequence(const char *json, uint32_t &sequence) {
  if (json == nullptr) {
    return false;
  }

  const char *sequenceStart = strstr(json, "\"s\":");
  if (sequenceStart == nullptr) {
    return false;
  }

  sequenceStart += 4;
  char *endPointer = nullptr;
  const unsigned long parsedValue = strtoul(sequenceStart, &endPointer, 10);
  if (endPointer == sequenceStart) {
    return false;
  }

  sequence = static_cast<uint32_t>(parsedValue);
  return true;
}

bool tryExtractCounterResetSequence(const char *json, uint32_t &sequence) {
  if (json == nullptr) {
    return false;
  }

  const char *sequenceStart = strstr(json, "\"q\":");
  if (sequenceStart == nullptr) {
    return false;
  }

  sequenceStart += 4;
  char *endPointer = nullptr;
  const unsigned long parsedValue = strtoul(sequenceStart, &endPointer, 10);
  if (endPointer == sequenceStart) {
    return false;
  }

  sequence = static_cast<uint32_t>(parsedValue);
  return true;
}

void resetBridgeMetrics(uint32_t handledSequence) {
  serialFramesReceived = 0;
  espNowTxAttempts = 0;
  espNowTxOk = 0;
  espNowTxFail = 0;
  oversizeDrops = 0;
  lastTxSeq = 0;
  lastTxLatencyUs = 0;
  lastHandledCounterResetSequence = handledSequence;
  lastBridgeStatusMs = millis();
  lastAttemptWindowMs = lastBridgeStatusMs;
  lastAttemptWindowCount = 0;
}

void maybeResetBridgeMetrics(const char *json) {
  uint32_t requestedResetSequence = 0;
  if (
    tryExtractCounterResetSequence(json, requestedResetSequence)
    && requestedResetSequence > lastHandledCounterResetSequence
  ) {
    resetBridgeMetrics(requestedResetSequence);
  }
}

void sendBridgeStatus() {
  const unsigned long nowMs = millis();
  if ((nowMs - lastBridgeStatusMs) < BRIDGE_STATUS_INTERVAL_MS) {
    return;
  }

  const float elapsedSeconds = lastAttemptWindowMs > 0
    ? static_cast<float>(nowMs - lastAttemptWindowMs) / 1000.0f
    : 0.0f;
  const uint32_t attemptsSinceWindow = espNowTxAttempts - lastAttemptWindowCount;
  const float txRateHz = elapsedSeconds > 0.0f
    ? static_cast<float>(attemptsSinceWindow) / elapsedSeconds
    : 0.0f;

  Serial.printf(
    "!{\"t\":\"bridge\",\"sr\":%lu,\"ea\":%lu,\"eo\":%lu,\"ef\":%lu,\"od\":%lu,\"thz\":%.2f,\"sq\":%lu,\"lu\":%lu,\"rq\":%lu}\n",
    static_cast<unsigned long>(serialFramesReceived),
    static_cast<unsigned long>(espNowTxAttempts),
    static_cast<unsigned long>(espNowTxOk),
    static_cast<unsigned long>(espNowTxFail),
    static_cast<unsigned long>(oversizeDrops),
    txRateHz,
    static_cast<unsigned long>(lastTxSeq),
    static_cast<unsigned long>(lastTxLatencyUs),
    static_cast<unsigned long>(lastHandledCounterResetSequence)
  );

  lastBridgeStatusMs = nowMs;
  lastAttemptWindowMs = nowMs;
  lastAttemptWindowCount = espNowTxAttempts;
}

bool initEspNowPeer() {
  Serial.println("Configuring WiFi station mode...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.println("Initialising ESP-NOW transmitter...");
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed.");
    return false;
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, DRONE_MAC_ADDRESS, sizeof(DRONE_MAC_ADDRESS));
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_now_del_peer(DRONE_MAC_ADDRESS);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer.");
    return false;
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataReceived);
  Serial.print("ESP-NOW peer added: ");
  for (size_t i = 0; i < sizeof(DRONE_MAC_ADDRESS); i++) {
    if (i > 0) {
      Serial.print(":");
    }
    if (DRONE_MAC_ADDRESS[i] < 16) {
      Serial.print("0");
    }
    Serial.print(DRONE_MAC_ADDRESS[i], HEX);
  }
  Serial.println();
  return true;
}

void sendLineOverEspNow(const String &line) {
  if (line.length() > MAX_JSON_PAYLOAD_LENGTH) {
    oversizeDrops += 1;
    Serial.print("Dropping oversized JSON payload: ");
    Serial.print(line.length());
    Serial.print(" bytes (limit ");
    Serial.print(MAX_JSON_PAYLOAD_LENGTH);
    Serial.println(").");
    return;
  }

  EspNowMessage message = {};
  const size_t copyLength = min(line.length(), MAX_JSON_PAYLOAD_LENGTH);
  line.substring(0, copyLength).toCharArray(message.payload, sizeof(message.payload));
  tryExtractSequence(message.payload, lastTxSeq);

  lastTxStartMicros = micros();
  espNowTxAttempts += 1;
  const esp_err_t result = esp_now_send(
    DRONE_MAC_ADDRESS,
    reinterpret_cast<const uint8_t *>(&message),
    copyLength
  );
  if (result != ESP_OK) {
    espNowTxFail += 1;
  }
}

void processIncomingFrame() {
  if (incomingFrameOverflow || incomingFrameLength <= 1) {
    incomingFrameLength = 0;
    incomingFrameOverflow = false;
    return;
  }

  incomingFrame[incomingFrameLength] = '\0';
  const int droneIndex = incomingFrame[0] - '0';
  if (droneIndex == 0) {
    maybeResetBridgeMetrics(incomingFrame + 1);
    serialFramesReceived += 1;
    sendLineOverEspNow(String(incomingFrame + 1));
  }

  incomingFrameLength = 0;
  incomingFrameOverflow = false;
}

void setup() {
  Serial.begin(USB_BAUD_RATE);

  delay(250);
  Serial.println();
  Serial.println("ESP32-S3 setup starting...");
  WiFi.mode(WIFI_STA);
  Serial.print("ESP32-S3 MAC: ");
  Serial.println(WiFi.macAddress());

  if (initEspNowPeer()) {
    Serial.println("ESP32-S3 sender ready. Waiting for USB serial payloads.");
  } else {
    Serial.println("ESP32-S3 sender failed to initialise ESP-NOW.");
  }
  lastBridgeStatusMs = millis();
  lastAttemptWindowMs = lastBridgeStatusMs;
}

void loop() {
  while (Serial.available() > 0) {
    const char incomingByte = static_cast<char>(Serial.read());
    if (incomingByte == '\r') {
      continue;
    }
    if (incomingByte == '\n') {
      processIncomingFrame();
      continue;
    }

    if (incomingFrameOverflow) {
      continue;
    }

    if (incomingFrameLength >= MAX_SERIAL_FRAME_LENGTH) {
      incomingFrameOverflow = true;
      continue;
    }

    incomingFrame[incomingFrameLength++] = incomingByte;
  }

  sendBridgeStatus();
  yield();
}
