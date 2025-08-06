#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <vector>
#include <Update.h>
#include <esp_partition.h>
#include <set>

// Bluetooth libraries
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"

// --- Config ---
const char* ssid = "loranet";
const char* password = "1qaz2wsx";
const char* mqtt_server = "mqttiot.loranet.my";
const int mqtt_port = 1885;
const char* mqtt_user = "iotdbuser";
const char* mqtt_pass = "IoTdb2024";
const char* mqtt_topic = "traffic/bluetooth/mac";
const char* mqtt_ota_topic = "traffic/bluetooth/ota";
const char* mqtt_control_topic = "traffic/bluetooth/control";
const char* point = "B"; // Change as needed

// --- Globals ---
WiFiClient espClient;
PubSubClient client(espClient);
String selfMac;

// Task handles
TaskHandle_t mqttTaskHandle = NULL;
TaskHandle_t bluetoothTaskHandle = NULL;

// OTA variables
bool otaInProgress = false;
size_t otaTotalSize = 0;
size_t otaReceivedSize = 0;
String otaChecksum = "";
const size_t OTA_CHUNK_SIZE = 1024;

// Bluetooth scanning variables
bool btScanning = false;
unsigned long lastScanTime = 0;
unsigned long lastPublishTime = 0;
const unsigned long SCAN_DURATION = 10000; // 10 seconds scan duration
const unsigned long SCAN_INTERVAL = 1000;  // 1 second between scans
const unsigned long PUBLISH_INTERVAL = 20000; // Publish every 20 seconds
std::set<String> discoveredDevices; // Use set to automatically remove duplicates
std::set<String> lastPublishedDevices;
bool isScanning = false;



// --- Get Device MAC ---
String getMacAddress() {
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  char macStr[18];
  sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
          mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(macStr);
}

// --- Get Current Time String ---
String getCurrentTimeString() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "Unknown";
  char timeStr[64];
  strftime(timeStr, sizeof(timeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);
  return String(timeStr);
}



// --- Bluetooth Callback Function ---
void btCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
  if (event == ESP_BT_GAP_DISC_RES_EVT) {
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            param->disc_res.bda[0], param->disc_res.bda[1], param->disc_res.bda[2],
            param->disc_res.bda[3], param->disc_res.bda[4], param->disc_res.bda[5]);
    String macString = String(macStr);
    
    // Just add to discovered devices set (automatically handles duplicates)
    discoveredDevices.insert(macString);
    Serial.println("[BLE] Found device: " + macString);
  }

  if (event == ESP_BT_GAP_DISC_STATE_CHANGED_EVT) {
    if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
      Serial.println("[BLE] Scan stopped");
      isScanning = false;
      lastScanTime = millis();
    }
  }
}

// --- Initialize Bluetooth ---
void initBluetooth() {
  Serial.println("[BLE] Initializing Bluetooth...");
  if (!btStart()) {
    Serial.println("[BLE] Failed to start Bluetooth");
    return;
  }

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_bt_controller_init(&bt_cfg);
  esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
  esp_bluedroid_init();
  esp_bluedroid_enable();
  esp_bt_gap_register_callback(btCallback);
  esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

  Serial.println("[BLE] Bluetooth initialized successfully");
}

// --- Start Bluetooth Scan ---
void startBluetoothScan() {
  if (!isScanning) {
    Serial.printf("[BLE] Starting scan for %d seconds...\n", SCAN_DURATION / 1000);
    discoveredDevices.clear(); // Clear previous results
    isScanning = true;
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, SCAN_DURATION / 1000, 0);
  }
}

// --- Bluetooth Scanning Function ---
void scanBluetoothDevices() {
  // Start scan if not currently scanning and enough time has passed
  if (!isScanning && (millis() - lastScanTime) >= SCAN_INTERVAL) {
    startBluetoothScan();
  }
}

// --- Publish Bluetooth Data ---
void publishBluetoothData() {
  if (discoveredDevices.empty()) {
    Serial.println("[MQTT] No devices found to publish");
    return;
  }
  
  // Check if we have new devices to publish
  if (discoveredDevices == lastPublishedDevices) {
    Serial.println("[MQTT] No new devices to publish");
    return;
  }
  
  StaticJsonDocument<2048> doc; // Increased size for more devices
  time_t current_time = time(nullptr);
  
  // Add main fields
  doc["scanner_id"] = selfMac;
  doc["location"] = point;
  doc["epoch"] = current_time;
  
  // Create detected devices array
  JsonArray devicesArray = doc.createNestedArray("detected_devices");
  for (const String& device : discoveredDevices) {
    JsonObject deviceObj = devicesArray.createNestedObject();
    deviceObj["bleid"] = device;
    deviceObj["rssi"] = -75; // Default RSSI since historical value not stored
    deviceObj["seen_at"] = current_time;
  }
  
  Serial.printf("[MQTT] Publishing %d unique devices\n", discoveredDevices.size());
  
  String jsonPayload;
  serializeJson(doc, jsonPayload);
  
  Serial.println("=== Publishing Bluetooth Data ===");
  Serial.println("JSON Payload: " + jsonPayload);
  Serial.printf("Devices found: %d\n", discoveredDevices.size());
  Serial.printf("Device MAC (ID): %s\n", selfMac.c_str());
  Serial.printf("Point: %s\n", point);
  Serial.printf("Time: %s\n", getCurrentTimeString().c_str());
  Serial.println("=================================");
  
  if (client.connected()) {
    if (client.publish(mqtt_topic, jsonPayload.c_str())) {
      Serial.println("✓ Bluetooth data published successfully");
      // Store last published devices to avoid duplicates
      lastPublishedDevices = discoveredDevices;
    } else {
      Serial.println("✗ Failed to publish Bluetooth data");
    }
  } else {
    Serial.println("✗ MQTT not connected");
  }
}

// --- MQTT Reconnect ---
void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      Serial.println("MQTT connected");
      // Subscribe to OTA and control topics
      client.subscribe(mqtt_ota_topic);
      client.subscribe(mqtt_control_topic);
    } else {
      delay(2000);
    }
  }
}

// --- NTP Setup ---
void setupTime() {
  configTime(28800, 0, "pool.ntp.org", "time.nist.gov");
  Serial.print("Waiting for time");
  while (time(nullptr) < 100000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("Time sync done.");
}

// --- OTA Callback ---
void otaCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.printf("[OTA] Received message: %s\n", message.c_str());
  
  if (message.startsWith("START:")) {
    // Parse START message: START:size:checksum
    int firstColon = message.indexOf(':');
    int secondColon = message.indexOf(':', firstColon + 1);
    
    if (firstColon != -1 && secondColon != -1) {
      otaTotalSize = message.substring(firstColon + 1, secondColon).toInt();
      otaChecksum = message.substring(secondColon + 1);
      otaReceivedSize = 0;
      otaInProgress = true;
      
      Serial.printf("[OTA] Starting update. Size: %d bytes, Checksum: %s\n", otaTotalSize, otaChecksum.c_str());
      
      // Begin update
      if (Update.begin(otaTotalSize)) {
        Serial.println("[OTA] Update.begin() successful");
      } else {
        Serial.println("[OTA] Update.begin() failed");
        otaInProgress = false;
      }
    }
  } else if (message == "END") {
    if (otaInProgress) {
      if (Update.end()) {
        Serial.println("[OTA] Update successful! Rebooting...");
        delay(1000);
        ESP.restart();
      } else {
        Serial.println("[OTA] Update failed!");
        otaInProgress = false;
      }
    }
  } else if (message == "CANCEL") {
    if (otaInProgress) {
      Update.abort();
      otaInProgress = false;
      Serial.println("[OTA] Update cancelled");
    }
  } else if (otaInProgress) {
    // Handle firmware data
    if (Update.write(payload, length) == length) {
      otaReceivedSize += length;
      int progress = (otaReceivedSize * 100) / otaTotalSize;
      Serial.printf("[OTA] Progress: %d/%d bytes (%d%%)\n", otaReceivedSize, otaTotalSize, progress);
    } else {
      Serial.println("[OTA] Write failed");
      Update.abort();
      otaInProgress = false;
    }
  }
}

// --- Control Callback ---
void controlCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.printf("[CONTROL] Received: %s\n", message.c_str());
  
  if (message == "reboot") {
    Serial.println("[CONTROL] Rebooting...");
    delay(1000);
    ESP.restart();
  } else if (message == "SCAN_STATUS") {
    // Send current status
    StaticJsonDocument<256> doc;
    doc["device_id"] = "esp32_ble_point_B";
    doc["status"] = "running";
    doc["uptime"] = millis();
    doc["free_heap"] = ESP.getFreeHeap();
    
    String jsonPayload;
    serializeJson(doc, jsonPayload);
    client.publish(mqtt_control_topic, jsonPayload.c_str());
  }
}

// --- MQTT Message Callback ---
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);
  
  if (topicStr == mqtt_ota_topic) {
    otaCallback(topic, payload, length);
  } else if (topicStr == mqtt_control_topic) {
    controlCallback(topic, payload, length);
  }
}

// --- Publish Specific JSON ---
void publishSpecificJSON() {
  StaticJsonDocument<256> doc;
  
  doc["device_id"] = "esp32_ble_point_B";
  doc["mac"] = "60:07:C4:42:04:86";
  doc["point"] = "B";
  doc["time"] = "Wednesday, August 06 2025 12:33:06 AM";

  String jsonPayload;
  serializeJson(doc, jsonPayload);
  
  Serial.println("Publishing: " + jsonPayload);
  
  if (client.connected()) {
    if (client.publish(mqtt_topic, jsonPayload.c_str())) {
      Serial.println("JSON published successfully");
    } else {
      Serial.println("Failed to publish JSON");
    }
  } else {
    Serial.println("MQTT not connected");
  }
}

// --- MQTT Task (Core 0) ---
void mqttTask(void *parameter) {
  Serial.println("MQTT Task started on core: " + String(xPortGetCoreID()));
  
  for(;;) {
    if (!client.connected()) {
      reconnect();
    }
    client.loop();
    
    // Keep MQTT connection alive
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// --- Bluetooth Task (Core 1) ---
void bluetoothTask(void *parameter) {
  Serial.println("Bluetooth Task started on core: " + String(xPortGetCoreID()));
  
  for(;;) {
    if (!otaInProgress) {
      // Scan for Bluetooth devices
      scanBluetoothDevices();
      
      // Check if it's time to publish (every 20 seconds)
      if ((millis() - lastPublishTime) >= PUBLISH_INTERVAL) {
        if (!discoveredDevices.empty()) {
          Serial.println("[BLE] Publishing collected devices...");
          publishBluetoothData();
          lastPublishTime = millis();
        }
      }
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Check every 1 second for better responsiveness
  }
}

// --- Setup ---
void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 BLE Point B - Starting up...");
  
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println("Connected to WiFi");

  // Setup time
  setupTime();

  // Setup MQTT
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(mqttCallback);
  selfMac = getMacAddress();
  Serial.println("Device MAC: " + selfMac);

  // Initialize Bluetooth
  initBluetooth();
  Serial.println("[BLE] Bluetooth scanning ready");
  
  // Create tasks for both cores
  xTaskCreatePinnedToCore(
    mqttTask,           // Task function
    "MQTTTask",         // Task name
    10000,              // Stack size (bytes)
    NULL,               // Task parameters
    1,                  // Task priority
    &mqttTaskHandle,    // Task handle
    0                   // Core 0
  );
  
  xTaskCreatePinnedToCore(
    bluetoothTask,      // Task function
    "BluetoothTask",    // Task name
    10000,              // Stack size (bytes)
    NULL,               // Task parameters
    1,                  // Task priority
    &bluetoothTaskHandle, // Task handle
    1                   // Core 1
  );
  
  Serial.println("Setup complete! Both cores are now running tasks.");
  Serial.println("OTA Update ready via MQTT topic: " + String(mqtt_ota_topic));
}

// --- Loop ---
void loop() {
  // Main loop is now minimal since tasks handle the work
  // You can add any additional logic here if needed
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}
