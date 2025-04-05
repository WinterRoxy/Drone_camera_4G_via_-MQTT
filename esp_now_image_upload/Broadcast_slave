#include <PPP.h>
#include <esp_now.h>
#include <WiFi.h>
#include <PubSubClient.h>

// PPP Configuration
#define PPP_MODEM_APN "m3-world"
#define PPP_MODEM_PIN ""
#define PPP_MODEM_RST     15
#define PPP_MODEM_RST_LOW false
#define PPP_MODEM_TX      17
#define PPP_MODEM_RX      16
#define PPP_MODEM_RTS     -1
#define PPP_MODEM_CTS     -1
#define PPP_MODEM_FC      ESP_MODEM_FLOW_CONTROL_NONE
#define PPP_MODEM_MODEL   PPP_MODEM_SIM7600
PPP.end()

// MQTT Info
const char* mqtt_server       = "test.mosquitto.org";
const int   mqtt_port         = 1883;
const char* mqtt_user         = "";
const char* mqtt_password     = "";
const char* mqtt_publish_topic= "drone/send/pic";

// Status LED
#define LED_PIN   2

// PPP & MQTT
NetworkClient networkClient;
PubSubClient mqttClient(networkClient);

// MAC address of the Master (Sender)
uint8_t senderMAC[] = {0xE0, 0x5A, 0x1B, 0xD1, 0x9C, 0xE0};

// Image buffer
#define MAX_IMAGE_SIZE 500000
uint8_t *incomingImage = NULL;
size_t incomingBytes = 0;
size_t expectedImageSize = 0;

//------------------------------------------------------------------------------
// ESP-NOW callback when data is received
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           info->src_addr[0], info->src_addr[1], info->src_addr[2],
           info->src_addr[3], info->src_addr[4], info->src_addr[5]);
  Serial.printf("Received %d bytes from %s\n", len, macStr);

  // Check if the packet is from the expected Master MAC
  bool fromSender = true;
  for (int i = 0; i < 6; i++) {
    if (info->src_addr[i] != senderMAC[i]) {
      fromSender = false;
      break;
    }
  }
  if (!fromSender){
    Serial.println("Packet not from expected Sender. Ignored.");
    return;
  }

  // 1) If len == 4 (sizeof(uint32_t)), this is the image size
  if (expectedImageSize == 0 && len == sizeof(uint32_t)) {
    memcpy(&expectedImageSize, incomingData, sizeof(uint32_t));
    Serial.printf("Expected Image Size: %u bytes\n", expectedImageSize);

    if (expectedImageSize > MAX_IMAGE_SIZE) {
      Serial.println("Image size exceeds maximum limit!");
      expectedImageSize = 0;
      return;
    }

    // Allocate memory
    incomingImage = (uint8_t*)malloc(expectedImageSize);
    if (incomingImage == NULL) {
      Serial.println("Failed to allocate memory for image");
      expectedImageSize = 0;
      return;
    }
    incomingBytes = 0; // start receiving
    return;
  }

  // 2) If expecting image data
  if (expectedImageSize > 0 && incomingImage != NULL) {
    size_t bytesRemaining = expectedImageSize - incomingBytes;
    size_t bytesToCopy = (len > bytesRemaining) ? bytesRemaining : len;

    memcpy(incomingImage + incomingBytes, incomingData, bytesToCopy);
    incomingBytes += bytesToCopy;

    Serial.printf("Received %zu/%u bytes\n", incomingBytes, expectedImageSize);

    if (incomingBytes >= expectedImageSize) {
      Serial.println("Image received completely!");

      // Send image via MQTT
      publishImageBinary();

      // Free memory
      free(incomingImage);
      incomingImage = NULL;
      incomingBytes = 0;
      expectedImageSize = 0;
    }
  }
}

//------------------------------------------------------------------------------
// Notify the Master (Sender) when ready or after upload
void notifyMaster(const char* msg) {
  esp_err_t result = esp_now_send(senderMAC, (const uint8_t*)msg, strlen(msg));
  if (result == ESP_OK) {
    Serial.printf("Notify Master: %s\n", msg);
  } else {
    Serial.printf("Error notifying Master: %s\n", msg);
  }
}

// Publish image data over MQTT
void publishImageBinary() {
  size_t segmentSize = 1024;
  for (size_t offset = 0; offset < expectedImageSize; offset += segmentSize) {
    size_t bytesToSend = (expectedImageSize - offset < segmentSize) 
                           ? (expectedImageSize - offset)
                           : segmentSize;
    if (mqttClient.connected()) {
      bool publishResult = mqttClient.publish(mqtt_publish_topic, 
                                              incomingImage + offset, 
                                              bytesToSend);
      if (publishResult) {
        Serial.printf("Published segment %zu-%zu successfully.\n",
                      offset, offset + bytesToSend - 1);
      } else {
        Serial.printf("Failed to publish segment %zu-%zu.\n",
                      offset, offset + bytesToSend - 1);
        break;
      }
    } else {
      Serial.println("MQTT not connected. Cannot publish image.");
      break;
    }
    delay(50);
  }

  // Notify Master after publishing the entire image
  notifyMaster("DONE");
}

//------------------------------------------------------------------------------
// Initialize ESP-NOW
void initESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, senderMAC, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;  // Important: specify ifidx

  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  Serial.println("ESP-NOW Initialized and Peer Added");
}

//------------------------------------------------------------------------------
// Initialize PPP
void setupPPP() {
  PPP.setApn(PPP_MODEM_APN);
  PPP.setPin(PPP_MODEM_PIN);
  PPP.setResetPin(PPP_MODEM_RST, PPP_MODEM_RST_LOW);
  PPP.setPins(PPP_MODEM_TX, PPP_MODEM_RX, PPP_MODEM_RTS, PPP_MODEM_CTS, PPP_MODEM_FC);

  delay(2000);
  Serial.println("\nStarting the modem. It might take a while!");
  PPP.begin(PPP_MODEM_MODEL);

  bool attached = PPP.attached();
  if (!attached) {
    int i = 0;
    Serial.print("Waiting to connect to network");
    while (!attached && (i < 60)) {
      Serial.print(".");
      delay(1000);
      attached = PPP.attached();
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      i++;
    }
  }

  if (attached) {
    Serial.println("\nConnected to network!");
    PPP.mode(ESP_MODEM_MODE_CMUX);
    if (!PPP.waitStatusBits(ESP_NETIF_CONNECTED_BIT, 10000)) {
      Serial.println("Failed to connect to internet!");
    } else {
      Serial.println("Connected to internet!");
    }
  } else {
    Serial.println("\nFailed to connect to network!");
  }
}

//------------------------------------------------------------------------------
// Connect to MQTT broker
void reconnect_mqtt() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");
    String clientId = "ESP32-Receiver-";
    clientId += String(random(0xffff), HEX);

    if (mqttClient.connect(clientId.c_str(), mqtt_user, mqtt_password)) {
      Serial.println("Connected");
      // Notify master that slave is ready (has MQTT connection)
      notifyMaster("READY");
    } else {
      Serial.print("Connection failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Trying again in 5 seconds");
      delay(5000);
    }
  }
}

//------------------------------------------------------------------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 Receiver Started");

  initESPNow();
  setupPPP();

  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setBufferSize(4096);

  reconnect_mqtt();
}

//------------------------------------------------------------------------------
void loop() {
  if (!mqttClient.connected()) {
    reconnect_mqtt();
  }
  mqttClient.loop();

  // Blink LED to indicate status
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  delay(200);
}
