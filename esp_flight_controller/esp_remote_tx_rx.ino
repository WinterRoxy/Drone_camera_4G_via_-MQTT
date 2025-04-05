#include <Arduino.h>
#include <HardwareSerial.h>
#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include <wifi_provisioning/manager.h>

#define DEFAULT_VOLTAGE 0.0
#define DEFAULT_ROLL 0.0
#define DEFAULT_PITCH 0.0

static uint8_t gpio_reset = 0;
bool wifi_connected = 0;

static float voltage = DEFAULT_VOLTAGE;
static float roll_ = DEFAULT_ROLL;
static float pitch_ = DEFAULT_PITCH;
static unsigned long lastUpdate = 0;
const char *service_name = "PROV_1357";
const char *pop = "abcd1234";
static int gpio_0 = 0;

Device  rollDevice("Roll", "custom.device.roll_");
Device  pitchDevice("Pitch", "custom.device.pitch_");
Device  batteryDevice("Battery", "custom.device.battery");

void sysProvEvent(arduino_event_t *sys_event)
{
  switch (sys_event->event_id) {
    case ARDUINO_EVENT_PROV_START:
#if CONFIG_IDF_TARGET_ESP32S3
        Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n", service_name, pop);
        printQR(service_name, pop, "ble");
#else
        Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on SoftAP\n", service_name, pop);
        printQR(service_name, pop, "softap");
#endif
        break;
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.printf("\nConnected to Wi-Fi!\n");
      wifi_connected = 1;
      delay(500);
      break;
    case ARDUINO_EVENT_PROV_CRED_RECV: {
        Serial.println("\nReceived Wi-Fi credentials");
        Serial.print("\tSSID : ");
        Serial.println((const char *) sys_event->event_info.prov_cred_recv.ssid);
        Serial.print("\tPassword : ");
        Serial.println((char const *) sys_event->event_info.prov_cred_recv.password);
        break;
      }
    case ARDUINO_EVENT_PROV_INIT:
      wifi_prov_mgr_disable_auto_stop(10000);
      break;
    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
      Serial.println("Stopping Provisioning!!!");
      wifi_prov_mgr_stop_provisioning();
      break;
  }
}

void write_callback(Device *device, Param *param, const param_val_t val, void *priv_data, write_ctx_t *ctx)
{
  const char *device_name = device->getDeviceName();
  const char *param_name = param->getParamName();

  if (strcmp(param_name, "Voltage") == 0) {
    Serial.printf("Received value = %0.2f for %s - %s\n", val.val.f, device_name, param_name);
    voltage = val.val.f;
    param->updateAndReport(val);
  }

  if (strcmp(param_name, "Roll_Val") == 0) {
    Serial.printf("Received value = %d for %s - %s\n", val.val.i, device_name, param_name);
    roll_=val.val.i;
    param->updateAndReport(val);
  }
  if (strcmp(param_name, "Pitch_Val") == 0) {
    Serial.printf("Received value = %d for %s - %s\n", val.val.i, device_name, param_name);
    pitch_ = val.val.i;
    param->updateAndReport(val);
  }
}

#define VRpitch 5
#define VRroll 4
#define VRthrottle 6
#define VRyaw 7
#define DROP_PIN 8

int throttle_read, pitch_read;
int yaw_read, roll_read;
int drop_val; 


const int RXpin = 19;
const int TXpin = 20;
HardwareSerial vietnam(2); // UART2

struct __attribute__((packed)) DataPacket {
  uint8_t startByte;
  int32_t throttle;
  int32_t pitch;
  int32_t roll;
  int32_t yaw;
  int32_t drop_val; 
  uint8_t endByte;
};

float pitch_recv = 0.0;
float roll_recv = 0.0;
float voltage_recv = 0.0;

struct __attribute__((packed)) DataPacketReceive {
  uint8_t startByte;
  float pitch_recev;
  float roll_recev;
  float voltage_recev;
  uint8_t endByte;
};

void sendData(int32_t throttle, int32_t pitch, int32_t roll, int32_t yaw, int32_t drop_val) {
  DataPacket packet;
  packet.startByte = 0xAA; 
  packet.throttle = throttle;
  packet.pitch = pitch;
  packet.roll = roll;
  packet.yaw = yaw;
  packet.drop_val = drop_val; 
  packet.endByte = 0x55; 
  vietnam.write((uint8_t*)&packet, sizeof(packet)); 
}

void receiveData() {
  static DataPacketReceive packet;
  static uint8_t *ptr = (uint8_t*)&packet;
  static size_t index = 0;
  static bool receiving = false;

  while (vietnam.available()) {
    uint8_t byteReceived = vietnam.read();

    if (!receiving) {
      if (byteReceived == 0xAB) { 
        ptr[0] = byteReceived;
        index = 1;
        receiving = true;
      }
    } else {
      ptr[index++] = byteReceived;

      if (index == sizeof(DataPacketReceive)) {
        receiving = false;
        index = 0;

        if (packet.endByte == 0xBA) {
          pitch_recv = packet.pitch_recev;
          roll_recv = packet.roll_recev;
          voltage_recv = packet.voltage_recev;

          Serial.println("Received Data:");
          Serial.print("Pitch Received: "); Serial.println(pitch_recv);
          Serial.print("Roll Received: ");  Serial.println(roll_recv);
          Serial.print("Voltage Received: "); Serial.println(voltage_recv);
        } else {
          Serial.println("Invalid end byte in received data");
        }
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(gpio_reset, INPUT);
  analogSetAttenuation(ADC_11db);
  vietnam.begin(115200, SERIAL_8N1, RXpin, TXpin);
  pinMode(DROP_PIN, INPUT_PULLUP);

  Node my_node = RMaker.initNode("Droneeee");

  Param voltageParam("Voltage (V)", "custom.param.voltage", value((float)DEFAULT_VOLTAGE), PROP_FLAG_READ );
  batteryDevice.addParam(voltageParam);
  batteryDevice.assignPrimaryParam(batteryDevice.getParamByName("Voltage (V)"));

  Param rollParam("Roll_Val", "custom.param.roll", value((float)DEFAULT_ROLL), PROP_FLAG_READ );
  rollDevice.addParam(rollParam);
  rollDevice.assignPrimaryParam(rollDevice.getParamByName("Roll_Val"));

  Param pitchParam("Pitch_Val", "custom.param.pitch", value((float)DEFAULT_PITCH), PROP_FLAG_READ );
  pitchDevice.addParam(pitchParam);
  pitchDevice.assignPrimaryParam(pitchDevice.getParamByName("Pitch_Val"));

  rollDevice.addCb(write_callback);
  pitchDevice.addCb(write_callback);
  batteryDevice.addCb(write_callback);

  my_node.addDevice(batteryDevice);
  my_node.addDevice(pitchDevice);
  my_node.addDevice(rollDevice);

  RMaker.enableOTA(OTA_USING_PARAMS);
  RMaker.enableTZService();
  RMaker.enableSchedule();

  Serial.printf("\nStarting ESP-RainMaker\n");
  RMaker.start();

  WiFi.onEvent(sysProvEvent);
#if CONFIG_IDF_TARGET_ESP32S3
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM, WIFI_PROV_SECURITY_1, pop, service_name);
#else
  WiFiProv.beginProvision(WIFI_PROV_SCHEME_SOFTAP, WIFI_PROV_SCHEME_HANDLER_NONE, WIFI_PROV_SECURITY_1, pop, service_name);
#endif

  Serial.println("Setup done.");
}

void loop() {

  if (digitalRead(gpio_reset) == LOW) { 
    Serial.printf("Reset Button Pressed!\n");

    delay(100);
    int startTime = millis();
    while (digitalRead(gpio_reset) == LOW) delay(50);
    int endTime = millis();

    if ((endTime - startTime) > 10000) {
      Serial.printf("Reset to factory.\n");
      wifi_connected = 0;
      RMakerFactoryReset(2);
    } else if ((endTime - startTime) > 3000) {
      Serial.printf("Reset Wi-Fi.\n");
      wifi_connected = 0;
      RMakerWiFiReset(2);
    }
  }
  pitch_read = analogRead(VRpitch);
  roll_read = analogRead(VRroll);
  throttle_read = analogRead(VRthrottle);
  yaw_read = analogRead(VRyaw);

  pitch_read = map(pitch_read, 297, 3898, 1000, 2000);
  roll_read = map(roll_read, 301, 4019, 2000, 1000);
  throttle_read = map(throttle_read, 275, 3648, 1000, 2000);
  yaw_read = map(yaw_read, 262, 3995, 2000, 1000);

  if((roll_read > 1535) && (roll_read < 1550)) roll_read = 1500;
  if((pitch_read > 1470) && (pitch_read < 1480)) pitch_read = 1500;
  if((yaw_read > 1530) && (yaw_read < 1545)) yaw_read = 1500;

  if (digitalRead(DROP_PIN) == LOW) {
    drop_val = 1;
  } else {
    drop_val = 0;
  }

  Serial.print("pitch = ");
  Serial.println(pitch_read);
  Serial.print("Roll = ");
  Serial.println(roll_read);
  Serial.print("Throttle = ");
  Serial.println(throttle_read);
  Serial.print("Yaw = ");
  Serial.println(yaw_read);
  Serial.print("Drop_val = ");
  Serial.println(drop_val);

  sendData(throttle_read, pitch_read, roll_read, yaw_read, drop_val);

  receiveData();

  pitch_ = pitch_recv;
  roll_ = roll_recv;
  voltage = voltage_recv;

  unsigned long currentMillis = millis();
  if (currentMillis - lastUpdate >= 1000) {
    batteryDevice.updateAndReportParam("Voltage (V)", voltage);
    rollDevice.updateAndReportParam("Roll_Val", roll_);
    pitchDevice.updateAndReportParam("Pitch_Val", pitch_);
    lastUpdate = currentMillis;
  }
  delay(50);
}
