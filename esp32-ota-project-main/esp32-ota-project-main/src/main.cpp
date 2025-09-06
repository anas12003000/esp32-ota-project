#if !defined(POINT_DEFINED)
#define POINT_DEFINED
const char* point = "A";
#endif
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <set>
#include <vector>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <esp_mac.h>
#include <esp_task_wdt.h>

// WiFi Configuration - Multiple SSIDs
struct WiFiCredential {
    const char* ssid;
    const char* password;
};

WiFiCredential wifiList[] = {
    {"radar", "pajero999"},
    {"loranet_HQ", "pajero999"},
    {"loranet", "1qaz2wsx"},
    {"anas", "pajero999"}
};
const int WIFI_LIST_SIZE = sizeof(wifiList) / sizeof(wifiList[0]);
int currentWiFiIndex = 0;

// MQTT Configuration
const char* mqtt_server = "mqttiot.loranet.my";
const int mqtt_port = 1885;
const char* mqtt_user = "iotdbuser";
const char* mqtt_pass = "IoTdb2024";
const char* mqtt_topic = "traffic/bluetooth/mac";


// Scanning configuration
const unsigned long SCAN_DURATION = 30000;  // 30 seconds
const unsigned long SCAN_INTERVAL = 30000;  // 30 seconds
const int WDT_TIMEOUT = 60;  // Watchdog timeout in seconds

// Safety configuration
const int MAX_WIFI_RECONNECT_ATTEMPTS = 5;
const int MAX_MQTT_RECONNECT_ATTEMPTS = 3;
const unsigned long SAFETY_CHECK_INTERVAL = 5000;  // Check every 5 seconds
const unsigned long MAX_LOOP_TIME = 60000;  // Max time without completing a cycle (1 minute)
const unsigned long MAX_WIFI_DISCONNECT_TIME = 30000;  // Max WiFi disconnect time (30 seconds)
const unsigned long AUTO_RESET_INTERVAL = 43200000;  // 12 hours in milliseconds (12 * 60 * 60 * 1000)

// Device info structure
class DeviceInfo {
public:
    String mac;
    int rssi;
    time_t seen_at;
    
    DeviceInfo() : rssi(0), seen_at(0) {}
    DeviceInfo(const String& m, int r, time_t t) : mac(m), rssi(r), seen_at(t) {}
    
    bool operator<(const DeviceInfo& other) const {
        return mac < other.mac;
    }
};

// Global variables
BLEScan* pBLEScan = nullptr;
std::set<DeviceInfo> discoveredDevices;
std::set<String> publishedDevices;  // Track published MACs to avoid duplicates
WiFiClient espClient;
PubSubClient client(espClient);
String selfMac;

// State tracking
bool isScanning = false;
unsigned long lastScanTime = 0;
unsigned long scanStartTime = 0;
unsigned long lastWatchdogReset = 0;

// Safety state tracking
unsigned long lastCycleComplete = 0;
unsigned long wifiDisconnectTime = 0;
unsigned long lastSafetyCheck = 0;
unsigned long systemStartTime = 0;  // Track system uptime for 12-hour reset
int wifiReconnectAttempts = 0;
int mqttReconnectAttempts = 0;
bool wifiConnectedLastCheck = true;

// Get MAC address
String getMacAddress() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char macStr[18];
    sprintf(macStr, "%02X:%02X:%02X:%02X:%02X:%02X",
            mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return String(macStr);
}

// BLE Scan Callback
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        String mac = advertisedDevice.getAddress().toString().c_str();
        int rssi = advertisedDevice.getRSSI();
        time_t seen_at = time(nullptr);
        DeviceInfo device(mac, rssi, seen_at);
        discoveredDevices.insert(device);
        Serial.printf("[BLE] Found: %s, RSSI: %d\n", mac.c_str(), rssi);
    }
};

// WiFi connection with retry logic
void connectWiFi() {
    bool connected = false;
    Serial.println("[WiFi] Scanning for available networks...");
    int n = WiFi.scanNetworks();
    if (n == 0) {
        Serial.println("[WiFi] No networks found!");
    } else {
        Serial.printf("[WiFi] %d networks found\n", n);
        for (int i = 0; i < WIFI_LIST_SIZE; ++i) {
            bool found = false;
            for (int j = 0; j < n; ++j) {
                String foundSSID = WiFi.SSID(j);
                if (foundSSID == wifiList[i].ssid) {
                    found = true;
                    break;
                }
            }
            if (found) {
                currentWiFiIndex = i;
                WiFi.begin(wifiList[i].ssid, wifiList[i].password);
                Serial.printf("[WiFi] Connecting to %s", wifiList[i].ssid);
                int attempts = 0;
                while (WiFi.status() != WL_CONNECTED && attempts < 20) {
                    delay(500);
                    Serial.print(".");
                    attempts++;
                    esp_task_wdt_reset();
                }
                if (WiFi.status() == WL_CONNECTED) {
                    Serial.println();
                    Serial.printf("[WiFi] Connected! SSID: %s, IP: %s\n", wifiList[i].ssid, WiFi.localIP().toString().c_str());
                    wifiReconnectAttempts = 0;
                    wifiConnectedLastCheck = true;
                    wifiDisconnectTime = 0;
                    connected = true;
                    break;
                } else {
                    Serial.println("\n[WiFi] Connection failed!");
                }
            } else {
                Serial.printf("[WiFi] SSID %s not found in scan\n", wifiList[i].ssid);
            }
        }
    }
    if (!connected) {
        wifiReconnectAttempts++;
        if (wifiReconnectAttempts >= MAX_WIFI_RECONNECT_ATTEMPTS) {
            Serial.printf("[WiFi] Failed %d times - Rebooting...\n", wifiReconnectAttempts);
            ESP.restart();
        }
    }
}

// Robust WiFi reconnection
void handleWiFiReconnection() {
    if (WiFi.status() != WL_CONNECTED) {
        if (wifiConnectedLastCheck) {
            // WiFi just disconnected
            wifiDisconnectTime = millis();
            wifiConnectedLastCheck = false;
            Serial.println("[WiFi] Disconnected! Starting reconnection...");
        }
        
        // Check if WiFi has been disconnected too long
        if (millis() - wifiDisconnectTime > MAX_WIFI_DISCONNECT_TIME) {
            Serial.println("[WiFi] Disconnected too long - Force reboot!");
            ESP.restart();
        }
        
        // Try to reconnect
        Serial.printf("[WiFi] Reconnect attempt %d/%d\n", wifiReconnectAttempts + 1, MAX_WIFI_RECONNECT_ATTEMPTS);
        connectWiFi();
    } else {
        // WiFi is connected
        if (!wifiConnectedLastCheck) {
            Serial.println("[WiFi] Reconnected successfully!");
            wifiConnectedLastCheck = true;
            wifiDisconnectTime = 0;
        }
    }
}

// MQTT connection with retry logic
void connectMQTT() {
    client.setServer(mqtt_server, mqtt_port);
    client.setBufferSize(8192); // Increase buffer size even more
    client.setKeepAlive(60);     // Set keepalive to 60 seconds
    client.setSocketTimeout(30); // Set socket timeout
    
    int attempts = 0;
    while (!client.connected() && attempts < 10) {
        Serial.printf("[MQTT] Connecting to %s:%d (attempt %d/10)\n", mqtt_server, mqtt_port, attempts + 1);
        
        String clientId = "ESP32-Point-A-" + selfMac;
        clientId.replace(":", "");
        
        if (client.connect(clientId.c_str(), mqtt_user, mqtt_pass)) {
            Serial.println("[MQTT] ✓ Connected successfully!");
            Serial.printf("[MQTT] Client ID: %s\n", clientId.c_str());
            Serial.printf("[MQTT] Buffer size: %d bytes\n", client.getBufferSize());
            mqttReconnectAttempts = 0;  // Reset counter on successful connection
            break;
        } else {
            int state = client.state();
            Serial.printf("[MQTT] ✗ Connection failed! State: %d ", state);
            switch (state) {
                case -4: Serial.println("(Connection timeout)"); break;
                case -3: Serial.println("(Connection lost)"); break;
                case -2: Serial.println("(Connect failed)"); break;
                case -1: Serial.println("(Disconnected)"); break;
                case 1: Serial.println("(Bad protocol)"); break;
                case 2: Serial.println("(Bad client ID)"); break;
                case 3: Serial.println("(Unavailable)"); break;
                case 4: Serial.println("(Bad credentials)"); break;
                case 5: Serial.println("(Unauthorized)"); break;
                default: Serial.println("(Unknown error)"); break;
            }
            attempts++;
            delay(2000);
            esp_task_wdt_reset(); // Reset watchdog during connection
        }
    }
    
    if (!client.connected()) {
        mqttReconnectAttempts++;
        Serial.printf("[MQTT] Failed to connect! Attempt %d/%d\n", mqttReconnectAttempts, MAX_MQTT_RECONNECT_ATTEMPTS);
        
        if (mqttReconnectAttempts >= MAX_MQTT_RECONNECT_ATTEMPTS) {
            Serial.println("[MQTT] Max reconnect attempts reached - Rebooting...");
            ESP.restart();
        }
    }
}

// Safety monitoring system
void performSafetyChecks() {
    unsigned long currentTime = millis();
    
    // Check for 12-hour auto-reset to prevent long-term system freeze
    if (currentTime - systemStartTime >= AUTO_RESET_INTERVAL) {
        Serial.println("[SAFETY] 12-hour auto-reset triggered - Rebooting for system refresh...");
        delay(1000);  // Give time for serial output
        ESP.restart();
    }
    
    // Check if main loop is hanging (no cycle completion for too long)
    if (lastCycleComplete > 0 && (currentTime - lastCycleComplete) > MAX_LOOP_TIME) {
        Serial.printf("[SAFETY] Loop hanging detected! Last cycle: %lu ms ago - Rebooting...\n", 
                     currentTime - lastCycleComplete);
        ESP.restart();
    }
    
    // Check WiFi connection stability
    handleWiFiReconnection();
    
    // Check MQTT connection
    if (WiFi.status() == WL_CONNECTED && !client.connected()) {
        Serial.println("[SAFETY] MQTT disconnected while WiFi is up - Reconnecting...");
        connectMQTT();
    }
    
    // Check memory health
    size_t freeHeap = ESP.getFreeHeap();
    if (freeHeap < 10000) {  // Less than 10KB free
        Serial.printf("[SAFETY] Low memory detected: %d bytes - Rebooting...\n", freeHeap);
        ESP.restart();
    }
    
    // Check if BLE scan is stuck
    if (isScanning && (currentTime - scanStartTime) > (SCAN_DURATION + 10000)) {
        Serial.println("[SAFETY] BLE scan stuck - Restarting scan...");
        try {
            pBLEScan->stop();
            pBLEScan->clearResults();
        } catch (...) {
            Serial.println("[SAFETY] Exception stopping stuck scan - Rebooting...");
            ESP.restart();
        }
        isScanning = false;
    }
}

// Publish Bluetooth data
void publishBluetoothData() {
    esp_task_wdt_reset(); // Reset watchdog before publishing
    
    // Check MQTT connection first
    if (!client.connected()) {
        Serial.println("[MQTT] ✗ Not connected! Cannot publish.");
        return;
    }
    
    // Get new devices that haven't been published yet
    std::set<DeviceInfo> newDevices;
    for (const DeviceInfo& device : discoveredDevices) {
        if (publishedDevices.find(device.mac) == publishedDevices.end()) {
            newDevices.insert(device);
        }
    }
    
    if (newDevices.empty()) {
        Serial.println("[MQTT] No new unique devices to publish");
        return;
    }
    
    Serial.printf("[MQTT] Publishing %d new unique devices\n", newDevices.size());
    
    // Create JSON payload with correct format
    StaticJsonDocument<4096> doc;  // Larger document for all devices
    doc["scanner_id"] = selfMac;
    doc["location"] = "A";
    doc["epoch"] = time(nullptr);
    
    JsonArray detectedDevices = doc.createNestedArray("detected_devices");
    for (const DeviceInfo& device : newDevices) {
        JsonObject deviceObj = detectedDevices.createNestedObject();
        deviceObj["mac"] = device.mac;  // Use "mac" as specified
        deviceObj["rssi"] = device.rssi;
        deviceObj["seen_at"] = device.seen_at;
    }
    
    String payload;
    size_t payloadSize = serializeJson(doc, payload);
    
    Serial.printf("[MQTT] Topic: %s\n", mqtt_topic);
    Serial.printf("[MQTT] Payload size: %d bytes\n", payloadSize);
    Serial.printf("[MQTT] JSON: %s\n", payload.c_str());
    
    // Check buffer size
    if (payloadSize >= client.getBufferSize()) {
        Serial.printf("[MQTT] ✗ Payload too large! Size: %d, Buffer: %d\n", payloadSize, client.getBufferSize());
        
        // If payload is too large, split into smaller batches
        const int MAX_DEVICES_PER_BATCH = 10;
        std::vector<DeviceInfo> deviceList(newDevices.begin(), newDevices.end());
        int totalBatches = (deviceList.size() + MAX_DEVICES_PER_BATCH - 1) / MAX_DEVICES_PER_BATCH;
        
        Serial.printf("[MQTT] Splitting into %d batches\n", totalBatches);
        
        for (int batch = 0; batch < totalBatches; batch++) {
            int startIdx = batch * MAX_DEVICES_PER_BATCH;
            int endIdx = min((int)deviceList.size(), startIdx + MAX_DEVICES_PER_BATCH);
            
            StaticJsonDocument<1024> batchDoc;
            batchDoc["scanner_id"] = selfMac;
            batchDoc["location"] = "A";
            batchDoc["epoch"] = time(nullptr);
            
            JsonArray batchDevices = batchDoc.createNestedArray("detected_devices");
            for (int i = startIdx; i < endIdx; i++) {
                JsonObject deviceObj = batchDevices.createNestedObject();
                deviceObj["mac"] = deviceList[i].mac;
                deviceObj["rssi"] = deviceList[i].rssi;
                deviceObj["seen_at"] = deviceList[i].seen_at;
            }
            
            String batchPayload;
            serializeJson(batchDoc, batchPayload);
            
            Serial.printf("[MQTT] Publishing batch %d/%d (%d devices)\n", batch + 1, totalBatches, endIdx - startIdx);
            
            if (client.publish(mqtt_topic, batchPayload.c_str())) {
                Serial.printf("[MQTT] ✓ Batch %d published successfully!\n", batch + 1);
                
                // Mark devices in this batch as published
                for (int i = startIdx; i < endIdx; i++) {
                    publishedDevices.insert(deviceList[i].mac);
                }
            } else {
                Serial.printf("[MQTT] ✗ Batch %d publish failed!\n", batch + 1);
            }
            
            delay(100);
            esp_task_wdt_reset();
        }
        return;
    }
    
    // Attempt to publish single payload
    bool publishResult = client.publish(mqtt_topic, payload.c_str());
    
    if (publishResult) {
        Serial.println("[MQTT] ✓ Published successfully!");
        
        // Mark all devices as published
        for (const DeviceInfo& device : newDevices) {
            publishedDevices.insert(device.mac);
        }
    } else {
        Serial.println("[MQTT] ✗ Publish failed!");
        Serial.printf("[MQTT] Client state: %d\n", client.state());
        Serial.printf("[MQTT] WiFi status: %d\n", WiFi.status());
        Serial.printf("[MQTT] Free heap: %d bytes\n", ESP.getFreeHeap());
        
        // Try to reconnect on next loop instead of rebooting immediately
        Serial.println("[MQTT] Will attempt reconnection on next cycle...");
    }
}

// Setup function
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("\n===================================");
    Serial.println("ESP32 Bluetooth Scanner - Point A");
    Serial.println("ESP32 Bluetooth Scanner - Point A");
    Serial.println("Continuous Scanning with Watchdog");
    Serial.println("===================================\n");
    
    // Initialize watchdog
    Serial.println("[WDT] Initializing watchdog timer...");
    esp_task_wdt_init(WDT_TIMEOUT, true); // Enable panic mode
    esp_task_wdt_add(NULL); // Add current task to watchdog
    esp_task_wdt_reset();
    Serial.printf("[WDT] Watchdog initialized with %d second timeout\n", WDT_TIMEOUT);
    
    // Get device MAC
    selfMac = getMacAddress();
    Serial.printf("[SETUP] Device MAC: %s\n", selfMac);
    
    // Connect to WiFi
    connectWiFi();
    
    // Setup time (simple)
    configTime(28800, 0, "pool.ntp.org");
    Serial.println("[TIME] Configuring NTP...");
    delay(2000);
    esp_task_wdt_reset();
    
    // Connect to MQTT
    connectMQTT();
    
    // Initialize BLE
    Serial.println("[BLE] Initializing BLE...");
    try {
        BLEDevice::init("ESP32_Scanner");
        pBLEScan = BLEDevice::getScan();
        
        if (pBLEScan == nullptr) {
            Serial.println("[BLE] Failed to get BLE scanner! Rebooting...");
            ESP.restart();
        }
        
        pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
        pBLEScan->setActiveScan(true);
        pBLEScan->setInterval(100);
        pBLEScan->setWindow(99);
        Serial.println("[BLE] ✓ BLE initialized successfully");
    } catch (...) {
        Serial.println("[BLE] Exception during BLE initialization! Rebooting...");
        ESP.restart();
    }
    
    Serial.println("\n[SETUP] Setup complete! Starting continuous scan cycle...\n");
    lastScanTime = millis();
    scanStartTime = millis();
    lastWatchdogReset = millis();
    lastCycleComplete = millis();  // Initialize safety timer
    lastSafetyCheck = millis();
    systemStartTime = millis();   // Initialize 12-hour reset timer
}

// Main loop
void loop() {
    unsigned long currentTime = millis();
    
    // Reset watchdog every 10 seconds
    if (currentTime - lastWatchdogReset >= 10000) {
        esp_task_wdt_reset();
        lastWatchdogReset = currentTime;
    }
    
    // Perform safety checks every 5 seconds
    if (currentTime - lastSafetyCheck >= SAFETY_CHECK_INTERVAL) {
        performSafetyChecks();
        lastSafetyCheck = currentTime;
    }
    
    // Check WiFi connection with robust reconnection
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] Connection lost! Attempting reconnection...");
        handleWiFiReconnection();
        return; // Skip this loop iteration
    }
    
    // Check MQTT connection and reconnect if needed
    if (!client.connected()) {
        Serial.println("[MQTT] Connection lost! Reconnecting...");
        connectMQTT();
        if (!client.connected()) {
            return; // Skip this loop iteration if still not connected
        }
    }
    client.loop(); // Process MQTT messages
    
    // Continuous scan and publish cycle - start new scan immediately after publish
    if (!isScanning) {
        // Start new scan cycle
        Serial.println("\n[CYCLE] === Starting 30-second scan cycle ===");
        discoveredDevices.clear(); // Clear previous devices before new scan
        isScanning = true;
        scanStartTime = currentTime;
        try {
            Serial.println("[BLE] Starting BLE scan for 30 seconds...");
            pBLEScan->start(30, false); // Non-blocking scan
            Serial.println("[BLE] ✓ BLE scan started");
        } catch (...) {
            Serial.println("[BLE] Exception during scan start! Rebooting...");
            ESP.restart();
        }
    }
    
    // Check if 30-second scan is complete
    if (isScanning && (currentTime - scanStartTime >= SCAN_DURATION)) {
        try {
            BLEScanResults foundDevices = pBLEScan->getResults();
            Serial.printf("[BLE] ✓ Scan complete. Found %d devices\n", foundDevices.getCount());
            pBLEScan->clearResults();
            isScanning = false;
            // Immediately publish unique devices
            publishBluetoothData();
            // Update cycle completion time for safety monitoring
            lastCycleComplete = currentTime;
            Serial.println("[CYCLE] === Cycle complete - Starting next scan ===\n");
            // Clear discoveredDevices after publishing to avoid memory leaks
            discoveredDevices.clear();
        } catch (...) {
            Serial.println("[BLE] Exception during scan completion! Rebooting...");
            ESP.restart();
        }
    }
    
    // Status update every 15 seconds
    static unsigned long lastStatus = 0;
    if (currentTime - lastStatus >= 15000) {
        unsigned long uptimeHours = (currentTime - systemStartTime) / 3600000;  // Convert to hours
        unsigned long uptimeMinutes = ((currentTime - systemStartTime) % 3600000) / 60000;  // Remaining minutes
        
        Serial.printf("[STATUS] WiFi: %s, MQTT: %s, Scanning: %s, Total unique: %d, Free heap: %d, Uptime: %luh %lum\n",
                     WiFi.status() == WL_CONNECTED ? "OK" : "FAIL",
                     client.connected() ? "OK" : "FAIL",
                     isScanning ? "YES" : "NO",
                     publishedDevices.size(),
                     ESP.getFreeHeap(),
                     uptimeHours,
                     uptimeMinutes);
        lastStatus = currentTime;
    }
    
    delay(100); // Small delay to prevent watchdog issues
}
