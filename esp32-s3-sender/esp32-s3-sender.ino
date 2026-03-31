#include <WiFi.h>
#include <esp_now.h>

constexpr unsigned long USB_BAUD_RATE = 1000000;
constexpr size_t MAX_JSON_PAYLOAD_LENGTH = 250;
constexpr size_t MAX_SERIAL_FRAME_LENGTH = MAX_JSON_PAYLOAD_LENGTH + 1;
constexpr unsigned long SERIAL_WAIT_MS = 2000;

uint8_t DRONE_MAC_ADDRESS[] = {0x60, 0x55, 0xF9, 0xDA, 0x4E, 0xD4};

struct EspNowMessage {
  char payload[MAX_JSON_PAYLOAD_LENGTH + 1];
};

char incomingFrame[MAX_SERIAL_FRAME_LENGTH + 1] = {};
size_t incomingFrameLength = 0;
bool incomingFrameOverflow = false;

void onDataSent(const wifi_tx_info_t *txInfo, esp_now_send_status_t status) {
  (void)txInfo;
  (void)status;
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

  const esp_err_t result = esp_now_send(
    DRONE_MAC_ADDRESS,
    reinterpret_cast<const uint8_t *>(&message),
    copyLength
  );
  (void)result;
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

  yield();
}
