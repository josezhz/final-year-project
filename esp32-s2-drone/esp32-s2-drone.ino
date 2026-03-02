#include <WiFi.h>
#include <esp_now.h>

constexpr unsigned long USB_BAUD_RATE = 115200;
constexpr size_t MAX_PAYLOAD_LENGTH = 240;
constexpr unsigned long SERIAL_WAIT_MS = 2000;
constexpr unsigned long HEARTBEAT_INTERVAL_MS = 3000;

struct EspNowMessage {
  char payload[MAX_PAYLOAD_LENGTH];
};

unsigned long lastHeartbeatMs = 0;

void onDataReceived(const esp_now_recv_info_t *recvInfo, const uint8_t *data, int dataLen) {
  (void)recvInfo;

  EspNowMessage message = {};
  const size_t copyLength = min(static_cast<size_t>(dataLen), sizeof(message.payload) - 1);
  memcpy(message.payload, data, copyLength);
  message.payload[copyLength] = '\0';

  Serial.print("Received from ESP32-S3: ");
  Serial.println(message.payload);
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

void setup() {
  Serial.begin(USB_BAUD_RATE);
  const unsigned long waitStart = millis();
  while (!Serial && (millis() - waitStart) < SERIAL_WAIT_MS) {
    delay(10);
  }

  delay(250);
  Serial.println();
  Serial.println("ESP32-S2 setup starting...");
  WiFi.mode(WIFI_STA);
  Serial.print("ESP32-S2 MAC: ");
  Serial.println(WiFi.macAddress());

  if (initEspNowReceiver()) {
    Serial.println("ESP32-S2 drone ready. Waiting for ESP-NOW payloads.");
  } else {
    Serial.println("ESP32-S2 drone failed to initialise ESP-NOW.");
  }
}

void loop() {
  if (millis() - lastHeartbeatMs >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeatMs = millis();
    Serial.println("ESP32-S2 heartbeat: waiting for payloads.");
  }

  delay(10);
}
