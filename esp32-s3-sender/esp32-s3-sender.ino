#include <WiFi.h>
#include <esp_now.h>

constexpr unsigned long USB_BAUD_RATE = 1000000;
constexpr size_t MAX_PAYLOAD_LENGTH = 240;
constexpr unsigned long SERIAL_WAIT_MS = 2000;

uint8_t DRONE_MAC_ADDRESS[] = {0x60, 0x55, 0xF9, 0xDA, 0x4E, 0xD4};

struct EspNowMessage {
  char payload[MAX_PAYLOAD_LENGTH];
};

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
  EspNowMessage message = {};
  const size_t copyLength = min(line.length(), MAX_PAYLOAD_LENGTH - 1);
  line.substring(0, copyLength).toCharArray(message.payload, MAX_PAYLOAD_LENGTH);

  const esp_err_t result = esp_now_send(
    DRONE_MAC_ADDRESS,
    reinterpret_cast<const uint8_t *>(&message),
    sizeof(message)
  );
  (void)result;
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
  const int availableBytes = Serial.available();
  if (availableBytes > 1) {
    const int droneIndex = Serial.read() - '0';
    char buffer[MAX_PAYLOAD_LENGTH] = {};
    const int payloadBytes = min(availableBytes - 1, static_cast<int>(MAX_PAYLOAD_LENGTH - 1));
    const int bytesRead = Serial.readBytes(buffer, payloadBytes);
    buffer[bytesRead] = '\0';

    if (droneIndex == 0 && bytesRead > 0) {
      sendLineOverEspNow(String(buffer));
    }
  } else {
    yield();
  }
}
