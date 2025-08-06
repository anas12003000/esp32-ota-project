/*
 * BLE Scanner Project
 * ESP32-based BLE scanner with MQTT and OTA capabilities
 */
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include "time.h"
#include <set>
#include <Update.h>

// === MQTT Broker Configuration ===
#define WIFI_SSID     "loranet"
#define WIFI_PASSWORD "1qaz2wsx"

#define MQTT_HOST     "mqttiot.loranet.my"
#define MQTT_PORT     1885
#define MQTT_USERNAME "iotdbuser"
#define MQTT_PASSWORD "IoTdb2024"
#define MQTT_TOPIC    "traffic/bluetooth/mac"
#define MQTT_CONTROL_TOPIC "traffic/bluetooth/control"
#define MQTT_OTA_TOPIC "traffic/bluetooth/ota"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"

WiFiClient espClient;
PubSubClient client(espClient);

// Malaysia Time UTC+8
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 8 * 3600;
const int daylightOffset_sec = 0;

std::set<String> seenMacs; // ✅ Store unique MACs

// OTA over MQTT variables
bool otaInProgress = false;
size_t otaTotalSize = 0;
size_t otaReceivedSize = 0;
String otaFirmwareData = "";

// Bluetooth scan timing variables
unsigned long scanDuration = 30; // Default 30 seconds
unsigned long scanInterval = 5;   // Default 5 seconds between scans
unsigned long lastScanTime = 0;
bool isScanning = false;

// Non-blocking timing variables
unsigned long lastWiFiCheck = 0;
unsigned long lastMQTTCheck = 0;
unsigned long lastTimeSync = 0;
unsigned long rebootTime = 0;
bool rebootPending = false;

// === Get formatted time ===
String getCurrentTimeString() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "Unknown";
  char timeStr[64];
  strftime(timeStr, sizeof(timeStr), "%A, %B %d %Y %I:%M:%S %p", &timeinfo);
  return String(timeStr);
}

// === Bluetooth Callback ===
void btCallback(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
  if (event == ESP_BT_GAP_DISC_RES_EVT) {
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            param->disc_res.bda[0], param->disc_res.bda[1], param->disc_res.bda[2],
            param->disc_res.bda[3], param->disc_res.bda[4], param->disc_res.bda[5]);
    String macString = String(macStr);

    if (seenMacs.find(macString) != seenMacs.end()) {
      // ✅ MAC already seen
      return;
    }

    seenMacs.insert(macString); // ✅ Store new MAC

    String jsonPayload = "{\"device_id\":\"esp32_ble_point_A\",";
    jsonPayload += "\"scanner_mac\":\"" + WiFi.macAddress() + "\",";
    jsonPayload += "\"scanner_ip\":\"" + WiFi.localIP().toString() + "\",";
    jsonPayload += "\"detected_mac\":\"" + macString + "\",";
    jsonPayload += "\"point\":\"A\",";
    jsonPayload += "\"time\":\"" + getCurrentTimeString() + "\"}";

    Serial.println("[BLE] Found MAC: " + jsonPayload);

    if (client.connected()) {
      client.publish(MQTT_TOPIC, jsonPayload.c_str());
      Serial.println("[MQTT] Published to topic.");
    } else {
      Serial.println("[MQTT] Not connected. Skipping publish.");
    }
  }

  if (event == ESP_BT_GAP_DISC_STATE_CHANGED_EVT) {
    if (param->disc_st_chg.state == ESP_BT_GAP_DISCOVERY_STOPPED) {
      Serial.println("[BLE] Scan stopped");
      isScanning = false;
      lastScanTime = millis();
    }
  }
}

// === WiFi Connection ===
bool connectToWiFi() {
  static int retries = 0;
  static bool connecting = false;
  static unsigned long lastRetry = 0;
  
  if (!connecting) {
    Serial.print("[WiFi] Connecting to WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    connecting = true;
    retries = 0;
    lastRetry = millis();
    return false;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WiFi] Connected!");
    Serial.print("[WiFi] IP: ");
    Serial.println(WiFi.localIP());
    connecting = false;
    retries = 0;
    return true;
  }
  
  if (millis() - lastRetry >= 500) {
    Serial.print(".");
    retries++;
    lastRetry = millis();
    
    if (retries >= 30) {
      Serial.println("\n[WiFi] ❌ Failed to connect.");
      connecting = false;
      retries = 0;
      return false;
    }
  }
  
  return false;
}

// === Time Sync ===
bool syncTime() {
  static int retries = 0;
  static bool syncing = false;
  static unsigned long lastRetry = 0;
  
  if (!syncing) {
    Serial.println("[TIME] Syncing...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    syncing = true;
    retries = 0;
    lastRetry = millis();
    return false;
  }
  
  struct tm timeinfo;
  if (getLocalTime(&timeinfo)) {
    Serial.println("\n[TIME] Time synchronized!");
    syncing = false;
    retries = 0;
    return true;
  }
  
  if (millis() - lastRetry >= 500) {
    Serial.print(".");
    retries++;
    lastRetry = millis();
    
    if (retries >= 20) {
      Serial.println("\n[TIME] ❌ Failed to sync time.");
      syncing = false;
      retries = 0;
      rebootPending = true;
      rebootTime = millis() + 1000;
      return false;
    }
  }
  
  return false;
}

// === MQTT Callback ===
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String topicStr = String(topic);
  
  if (topicStr == MQTT_OTA_TOPIC) {
    handleOTAMessage(payload, length);
    return;
  }
  
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  Serial.printf("[MQTT] Message received on topic %s: %s\n", topic, message.c_str());

  if (message == "reboot") {
    Serial.println("[MQTT] Reboot command received. Restarting...");
    rebootPending = true;
    rebootTime = millis() + 1000;
  }
  
  // Handle scan timing commands
  if (message.startsWith("SCAN_DURATION:")) {
    int newDuration = message.substring(13).toInt();
    if (newDuration > 0 && newDuration <= 300) { // Max 5 minutes
      scanDuration = newDuration;
      Serial.printf("[MQTT] Scan duration updated to %d seconds\n", scanDuration);
    } else {
      Serial.println("[MQTT] Invalid scan duration. Must be 1-300 seconds.");
    }
  }
  
  if (message.startsWith("SCAN_INTERVAL:")) {
    int newInterval = message.substring(14).toInt();
    if (newInterval > 0 && newInterval <= 60) { // Max 1 minute
      scanInterval = newInterval;
      Serial.printf("[MQTT] Scan interval updated to %d seconds\n", scanInterval);
    } else {
      Serial.println("[MQTT] Invalid scan interval. Must be 1-60 seconds.");
    }
  }
  
  if (message == "SCAN_START") {
    Serial.println("[MQTT] Manual scan start requested");
    if (!isScanning) {
      startBluetoothScan();
    }
  }
  
  if (message == "SCAN_STOP") {
    Serial.println("[MQTT] Manual scan stop requested");
    if (isScanning) {
      esp_bt_gap_cancel_discovery();
    }
  }
  
  if (message == "SCAN_STATUS") {
    String statusMsg = "{\"scanning\":" + String(isScanning ? "true" : "false") + 
                      ",\"duration\":" + String(scanDuration) + 
                      ",\"interval\":" + String(scanInterval) + "}";
    client.publish(MQTT_CONTROL_TOPIC, statusMsg.c_str());
    Serial.println("[MQTT] Scan status sent: " + statusMsg);
  }
}

// === OTA over MQTT Functions ===
void handleOTAMessage(byte* payload, unsigned int length) {
  // Check if this is a control message
  if (length < 10) {
    String controlMsg = "";
    for (int i = 0; i < length; i++) {
      controlMsg += (char)payload[i];
    }
    
    if (controlMsg.startsWith("START:")) {
      // Parse firmware size
      String sizeStr = controlMsg.substring(6);
      otaTotalSize = sizeStr.toInt();
      otaReceivedSize = 0;
      otaFirmwareData = "";
      otaInProgress = true;
      
      Serial.printf("[MQTT-OTA] Starting firmware update. Expected size: %d bytes\n", otaTotalSize);
      
      // Begin the update
      if (Update.begin(otaTotalSize)) {
        Serial.println("[MQTT-OTA] Update.begin() successful");
      } else {
        Serial.println("[MQTT-OTA] Update.begin() failed");
        otaInProgress = false;
      }
      return;
    }
    
    if (controlMsg == "END") {
      if (otaInProgress && otaReceivedSize == otaTotalSize) {
        Serial.println("[MQTT-OTA] Finalizing update...");
        if (Update.end()) {
          Serial.println("[MQTT-OTA] Update successful! Rebooting...");
          rebootPending = true;
          rebootTime = millis() + 1000;
        } else {
          Serial.println("[MQTT-OTA] Update failed!");
        }
      } else {
        Serial.printf("[MQTT-OTA] Update failed! Received: %d, Expected: %d\n", otaReceivedSize, otaTotalSize);
      }
      otaInProgress = false;
      return;
    }
    
    if (controlMsg == "CANCEL") {
      Serial.println("[MQTT-OTA] Update cancelled");
      otaInProgress = false;
      Update.abort();
      return;
    }
  }
  
  // Handle firmware data
  if (otaInProgress) {
    if (Update.write(payload, length) == length) {
      otaReceivedSize += length;
      Serial.printf("[MQTT-OTA] Progress: %d/%d bytes (%.1f%%)\n", 
                   otaReceivedSize, otaTotalSize, 
                   (float)otaReceivedSize * 100 / otaTotalSize);
    } else {
      Serial.println("[MQTT-OTA] Write failed!");
      otaInProgress = false;
      Update.abort();
    }
  }
}

// === MQTT Connect ===
bool connectToMQTT() {
  static int retryCount = 0;
  static bool connecting = false;
  static unsigned long lastRetry = 0;
  const int maxRetries = 5;

  if (!connecting) {
    Serial.print("[MQTT] Connecting...");
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("Connected to MQTT!");
      client.subscribe(MQTT_CONTROL_TOPIC);
      Serial.println("[MQTT] Subscribed to control topic.");
      client.subscribe(MQTT_OTA_TOPIC);
      Serial.println("[MQTT] Subscribed to OTA topic.");
      connecting = false;
      retryCount = 0;
      return true;
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      Serial.println("... retrying in 5 seconds");
      retryCount++;
      lastRetry = millis();
      connecting = true;
      return false;
    }
  }

  if (millis() - lastRetry >= 5000) {
    if (retryCount >= maxRetries) {
      Serial.println("[MQTT] ❌ Unable to connect after several attempts. Rebooting...");
      connecting = false;
      retryCount = 0;
      rebootPending = true;
      rebootTime = millis() + 2000;
      return false;
    }
    
    // Try to connect again
    String clientId = "ESP32Client-" + String(random(0xffff), HEX);
    if (client.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
      Serial.println("Connected to MQTT!");
      client.subscribe(MQTT_CONTROL_TOPIC);
      Serial.println("[MQTT] Subscribed to control topic.");
      client.subscribe(MQTT_OTA_TOPIC);
      Serial.println("[MQTT] Subscribed to OTA topic.");
      connecting = false;
      retryCount = 0;
      return true;
    } else {
      retryCount++;
      lastRetry = millis();
    }
  }

  return false;
}

// === OTA Setup ===
void setupOTA() {
  ArduinoOTA.setHostname("esp32_ble_point_A");
  ArduinoOTA.setPassword("MyStrongOTAPassword");

  ArduinoOTA.onStart([]() {
    Serial.println("[OTA] Start updating...");
  });

  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA] Update complete.");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });

  ArduinoOTA.begin();
  Serial.println("[OTA] Ready for OTA updates.");
}

// === Bluetooth Setup ===
void setupBluetooth() {
  Serial.println("[BLE] Initializing Bluetooth...");
  if (!btStart()) {
    Serial.println("[BLE] Failed to start");
    return;
  }

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  esp_bt_controller_init(&bt_cfg);
  esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
  esp_bluedroid_init();
  esp_bluedroid_enable();
  esp_bt_gap_register_callback(btCallback);
  esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);

  Serial.println("[BLE] Bluetooth initialized. Starting first scan...");
  startBluetoothScan();
}

// === Start Bluetooth Scan ===
void startBluetoothScan() {
  if (!isScanning) {
    Serial.printf("[BLE] Starting scan for %d seconds...\n", scanDuration);
    isScanning = true;
    esp_bt_gap_start_discovery(ESP_BT_INQ_MODE_GENERAL_INQUIRY, scanDuration, 0);
  }
}

// === Setup ===
void setup() {
  Serial.begin(115200);
  Serial.println("[BOOT] Starting ESP32 BLE scanner...");

  // Initialize MQTT
  client.setServer(MQTT_HOST, MQTT_PORT);
  client.setCallback(mqttCallback);
  
  setupOTA();
  Serial.print("[INFO] ESP32 MAC Address: ");
  Serial.println(WiFi.macAddress());

  setupBluetooth();
}

// === Loop ===
void loop() {
  ArduinoOTA.handle();

  // Handle WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWiFiCheck >= 1000) {
      Serial.println("[WiFi] Lost connection. Reconnecting...");
      lastWiFiCheck = millis();
    }
    connectToWiFi();
  } else {
    lastWiFiCheck = millis();
  }

  // Handle MQTT connection
  if (!client.connected()) {
    if (millis() - lastMQTTCheck >= 1000) {
      lastMQTTCheck = millis();
    }
    connectToMQTT();
  } else {
    lastMQTTCheck = millis();
    client.loop();
  }
  
  // Handle time sync (every 24 hours)
  if (millis() - lastTimeSync >= 24 * 60 * 60 * 1000) {
    syncTime();
    lastTimeSync = millis();
  }
  
  // Handle Bluetooth scan timing
  if (!isScanning && (millis() - lastScanTime) >= (scanInterval * 1000)) {
    startBluetoothScan();
  }
  
  // Handle pending reboot
  if (rebootPending && millis() >= rebootTime) {
    ESP.restart();
  }
} 