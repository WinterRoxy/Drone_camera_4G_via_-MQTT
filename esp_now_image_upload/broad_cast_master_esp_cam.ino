#include <esp_now.h>
#include <WiFi.h>
#include "esp_camera.h"

// Define the appropriate pins for your ESP32-CAM board
#define PWDN_GPIO_NUM    32
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    0
#define SIOD_GPIO_NUM    26
#define SIOC_GPIO_NUM    27

#define Y9_GPIO_NUM      35
#define Y8_GPIO_NUM      34
#define Y7_GPIO_NUM      39
#define Y6_GPIO_NUM      36
#define Y5_GPIO_NUM      21
#define Y4_GPIO_NUM      19
#define Y3_GPIO_NUM      18
#define Y2_GPIO_NUM      5
#define VSYNC_GPIO_NUM   25
#define HREF_GPIO_NUM    23
#define PCLK_GPIO_NUM    22

// MAC address of the Receiver (Slave)
uint8_t receiverMAC[] = {0xFC, 0xB4, 0x67, 0x56, 0xF9, 0x04};

// Flag indicating whether we are allowed to send image
bool canSend = false;

// Camera configuration
camera_config_t config = {
  .pin_pwdn = PWDN_GPIO_NUM,
  .pin_reset = RESET_GPIO_NUM,
  .pin_xclk = XCLK_GPIO_NUM,
  .pin_sccb_sda = SIOD_GPIO_NUM,
  .pin_sccb_scl = SIOC_GPIO_NUM,

  .pin_d7 = Y9_GPIO_NUM,
  .pin_d6 = Y8_GPIO_NUM,
  .pin_d5 = Y7_GPIO_NUM,
  .pin_d4 = Y6_GPIO_NUM,
  .pin_d3 = Y5_GPIO_NUM,
  .pin_d2 = Y4_GPIO_NUM,
  .pin_d1 = Y3_GPIO_NUM,
  .pin_d0 = Y2_GPIO_NUM,
  .pin_vsync = VSYNC_GPIO_NUM,
  .pin_href = HREF_GPIO_NUM,
  .pin_pclk = PCLK_GPIO_NUM,

  .xclk_freq_hz = 20000000,
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,
  .pixel_format = PIXFORMAT_JPEG,
  .frame_size = FRAMESIZE_SVGA, 
  .jpeg_quality = 10,  
  .fb_count = 1
};

// ESP-NOW send callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// ESP-NOW receive callback (Master receives message from Slave)
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Check if it's from the known Slave
  bool fromSlave = true;
  for (int i = 0; i < 6; i++) {
    if (mac[i] != receiverMAC[i]) {
      fromSlave = false;
      break;
    }
  }
  if (!fromSlave) {
    Serial.println("Received data from unknown device");
    return;
  }

  // Check content of the message
  if (len == 5 && memcmp(incomingData, "READY", 5) == 0) {
    canSend = true;
    Serial.println("Received 'READY' from Slave => Slave connected to MQTT and is ready!");
  }
  else if (len == 4 && memcmp(incomingData, "DONE", 4) == 0) {
    canSend = true;
    Serial.println("Received 'DONE' from Slave => Slave finished uploading, ready for next image!");
  }
  else {
    Serial.println("Received data from Slave but not recognized");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  delay(3000);

  // Initialize camera
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return;
  }

  // Set WiFi to STA mode
  WiFi.mode(WIFI_STA);
  WiFi.begin();
  WiFi.disconnect();

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callbacks
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);

  // Add peer (the Slave)
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  Serial.println("ESP-NOW Initialized and Peer Added");
}

void loop() {
  // Only send image when canSend = true
  if (!canSend) {
    // Not received READY/DONE => Slave not ready yet
    return;
  }
  // Lock sending after decision to wait for Slave's DONE
  canSend = false;

  // Capture image
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  Serial.printf("Captured Image Size: %zu bytes\n", fb->len);

  // First send the image size
  uint32_t imageSize = fb->len;
  esp_now_send(receiverMAC, (uint8_t*)&imageSize, sizeof(imageSize));
  delay(100); 

  // Send image data in chunks
  size_t totalBytes = fb->len;
  size_t bytesSent = 0;
  const uint8_t *data = fb->buf;

  while (bytesSent < totalBytes) {
    size_t chunkSize = (totalBytes - bytesSent > 240) ? 240 : (totalBytes - bytesSent);
    uint8_t buffer[240];
    memcpy(buffer, data + bytesSent, chunkSize);

    esp_err_t result = esp_now_send(receiverMAC, buffer, chunkSize);
    if (result == ESP_OK) {
      Serial.printf("Sent %zu bytes\n", chunkSize);
    } else {
      Serial.println("Error sending the data");
    }

    bytesSent += chunkSize;
    delay(10);
  }

  // Release camera buffer
  esp_camera_fb_return(fb);

  delay(10);
}
