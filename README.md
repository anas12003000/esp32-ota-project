# BLE Scanner Project

A comprehensive ESP32-based BLE scanner project that combines Bluetooth Low Energy (BLE) scanning, MQTT communication, and Over-The-Air (OTA) firmware updates. This project creates a smart IoT device that can detect nearby Bluetooth devices and send data to an MQTT broker, while supporting remote firmware updates.

## 🚀 Features

### Core Functionality
- **Bluetooth Device Scanning**: Continuously scans for nearby Bluetooth devices
- **MQTT Communication**: Publishes device data to MQTT broker
- **OTA Updates**: Remote firmware updates over MQTT
- **WiFi Management**: Non-blocking WiFi connection handling
- **Time Synchronization**: NTP time sync for accurate timestamps

### Advanced Features
- **Configurable Scanning**: Adjust scan duration and intervals via MQTT
- **Non-blocking Operations**: All operations use `millis()` instead of `delay()`
- **Remote Control**: Control device via MQTT commands
- **Duplicate Prevention**: Tracks unique MAC addresses to avoid spam
- **JSON Payloads**: Structured data with IP and MAC addresses

## 📋 Requirements

### Hardware
- ESP32 development board (recommended: ESP32 Wrover or ESP32-S3 for larger flash memory)
- USB cable for programming
- WiFi network access

### Software
- Arduino IDE or Arduino CLI
- ESP32 board package
- PubSubClient library

## 🛠️ Installation

### 1. Clone the Repository
```bash
git clone https://github.com/shmrymbd/ble-scanner.git
cd ble-scanner
```

### 2. Install Libraries
Run the provided installation script:
```bash
chmod +x install_libraries.sh
./install_libraries.sh
```

Or install manually:
```bash
# Install Arduino CLI (if not already installed)
brew install arduino-cli

# Install ESP32 board package
arduino-cli core install esp32:esp32

# Install PubSubClient library
arduino-cli lib install "PubSubClient"
```

### 3. Configure Settings
Edit the configuration section in `esp32-ota-project.ino`:

```cpp
// WiFi Configuration
#define WIFI_SSID     "your_wifi_ssid"
#define WIFI_PASSWORD "your_wifi_password"

// MQTT Configuration
#define MQTT_HOST     "your_mqtt_broker"
#define MQTT_PORT     1883
#define MQTT_USERNAME "your_mqtt_username"
#define MQTT_PASSWORD "your_mqtt_password"
#define MQTT_TOPIC    "traffic/bluetooth/mac"
#define MQTT_CONTROL_TOPIC "traffic/bluetooth/control"
#define MQTT_OTA_TOPIC "traffic/bluetooth/ota"
```

## 🔧 Compilation

### Using Arduino CLI
```bash
# For ESP32 Dev Module (may have memory issues)
arduino-cli compile --fqbn esp32:esp32:esp32 esp32-ota-project.ino

# For ESP32 Wrover (recommended - more memory)
arduino-cli compile --fqbn esp32:esp32:esp32wrover esp32-ota-project.ino

# For ESP32-S3 (best option - 8MB flash)
arduino-cli compile --fqbn esp32:esp32:esp32s3 esp32-ota-project.ino
```

### Using Arduino IDE
1. Open `esp32-ota-project.ino` in Arduino IDE
2. Select appropriate board (ESP32 Wrover recommended)
3. Click Upload

## 📡 MQTT Topics

### Data Publishing
- **Topic**: `traffic/bluetooth/mac`
- **Payload**: JSON with device information
```json
{
  "device_id": "esp32_ble_point_A",
  "scanner_mac": "AA:BB:CC:DD:EE:FF",
  "scanner_ip": "192.168.1.100",
  "detected_mac": "11:22:33:44:55:66",
  "point": "A",
  "time": "Monday, December 16 2024 02:30:45 PM"
}
```

### Control Commands
Send to `traffic/bluetooth/control`:

| Command | Description |
|---------|-------------|
| `reboot` | Restart the ESP32 |
| `SCAN_DURATION:60` | Set scan duration to 60 seconds |
| `SCAN_INTERVAL:30` | Set interval between scans to 30 seconds |
| `SCAN_START` | Start scanning immediately |
| `SCAN_STOP` | Stop current scan |
| `SCAN_STATUS` | Get current scan status |

### OTA Updates
Send firmware to `traffic/bluetooth/ota`:
1. Send `START:123456` (firmware size in bytes)
2. Send firmware data in chunks
3. Send `END` to finalize update

## ⚙️ Configuration

### Default Settings
```cpp
// Bluetooth scan timing
unsigned long scanDuration = 30; // 30 seconds per scan
unsigned long scanInterval = 5;   // 5 seconds between scans

// Time zone (Malaysia UTC+8)
const long gmtOffset_sec = 8 * 3600;
const int daylightOffset_sec = 0;
```

### OTA Settings
```cpp
// OTA configuration
ArduinoOTA.setHostname("esp32_ble_point_A");
ArduinoOTA.setPassword("MyStrongOTAPassword");
```

## 🔄 Usage Examples

### 1. Basic Operation
1. Upload code to ESP32
2. Device connects to WiFi automatically
3. Starts scanning for Bluetooth devices
4. Publishes detected devices to MQTT

### 2. Remote Control
```bash
# Check scan status
mosquitto_pub -h your_broker -t "traffic/bluetooth/control" -m "SCAN_STATUS"

# Change scan timing
mosquitto_pub -h your_broker -t "traffic/bluetooth/control" -m "SCAN_DURATION:60"
mosquitto_pub -h your_broker -t "traffic/bluetooth/control" -m "SCAN_INTERVAL:30"

# Reboot device
mosquitto_pub -h your_broker -t "traffic/bluetooth/control" -m "reboot"
```

### 3. OTA Firmware Update
```python
import paho.mqtt.publish as publish

def send_firmware_update(bin_file_path):
    with open(bin_file_path, 'rb') as f:
        firmware_data = f.read()
    
    # Send start command
    publish.single("traffic/bluetooth/ota", f"START:{len(firmware_data)}")
    
    # Send firmware in chunks
    chunk_size = 1024
    for i in range(0, len(firmware_data), chunk_size):
        chunk = firmware_data[i:i+chunk_size]
        publish.single("traffic/bluetooth/ota", chunk)
    
    # Send end command
    publish.single("traffic/bluetooth/ota", "END")
```

## 📊 Memory Usage

### Current Memory Requirements
- **Program size**: ~1.7MB (requires ESP32 with 4MB+ flash)
- **Dynamic memory**: ~65KB (19% of available)
- **Recommended board**: ESP32 Wrover or ESP32-S3

### Memory Optimization
If you encounter memory issues:
1. Remove debug Serial prints
2. Use PROGMEM for constant strings
3. Optimize JSON payload generation
4. Use ESP32 with larger flash memory

## 🐛 Troubleshooting

### Common Issues

**1. Memory Error**
```
Sketch too big; see https://support.arduino.cc/hc/en-us/articles/360013825179
```
**Solution**: Use ESP32 Wrover or ESP32-S3 board

**2. WiFi Connection Failed**
```
[WiFi] ❌ Failed to connect.
```
**Solution**: Check WiFi credentials and network availability

**3. MQTT Connection Failed**
```
[MQTT] ❌ Unable to connect after several attempts.
```
**Solution**: Verify MQTT broker settings and network connectivity

**4. Bluetooth Not Working**
```
[BLE] Failed to start
```
**Solution**: Ensure ESP32 supports Classic Bluetooth (not just BLE)

### Debug Information
The device provides detailed debug output via Serial:
- WiFi connection status
- MQTT connection status
- Bluetooth scan progress
- OTA update progress
- Error messages

## 📈 Performance

### Optimizations Implemented
- **Non-blocking operations**: No `delay()` calls
- **Efficient memory usage**: Smart variable management
- **Configurable timing**: Adjustable scan intervals
- **Duplicate prevention**: Prevents spam from same devices

### Expected Performance
- **Scan range**: ~10-30 meters (depending on environment)
- **Scan frequency**: Configurable (1-300 seconds)
- **MQTT publish rate**: Real-time as devices detected
- **OTA update time**: Depends on firmware size and network

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📄 License

This project is open source and available under the [MIT License](LICENSE).

## 🙏 Acknowledgments

- ESP32 Arduino core developers
- PubSubClient library maintainers
- Arduino OTA library contributors

---

**Happy IoT Development! 🚀** 