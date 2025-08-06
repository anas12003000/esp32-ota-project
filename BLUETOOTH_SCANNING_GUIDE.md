# ESP32 Bluetooth Scanning Guide

## Overview

This ESP32 project implements Bluetooth Low Energy (BLE) scanning functionality that discovers nearby devices and publishes the data via MQTT in a specific JSON format.

## JSON Data Structure

The ESP32 publishes Bluetooth scanning data in the following JSON format:

```json
{
  "devices": [
    "09:CE:BB:F9:3F:82",
    "41:BB:00:EA:66:88", 
    "6B:E1:C3:26:C7:95"
  ],
  "id": "60:07:C4:42:04:86",
  "point": "A",
  "time": 1754419376
}
```

### Field Descriptions

- **devices**: Array of discovered Bluetooth device MAC addresses
- **id**: The ESP32 device's own MAC address (used as device identifier)
- **point**: Location point identifier (configurable, default: "A")
- **time**: Unix timestamp when the scan was performed

## Configuration

### WiFi Settings
```cpp
const char* ssid = "loranet";
const char* password = "1qaz2wsx";
```

### MQTT Settings
```cpp
const char* mqtt_server = "mqttiot.loranet.my";
const int mqtt_port = 1885;
const char* mqtt_user = "iotdbuser";
const char* mqtt_pass = "IoTdb2024";
const char* mqtt_topic = "traffic/bluetooth/mac";
```

### Bluetooth Settings
```cpp
const char* point = "A"; // Change as needed
const unsigned long SCAN_INTERVAL = 10000; // Scan every 10 seconds
```

## Features

### 1. BLE Scanning
- Passive scanning mode for low power consumption
- Configurable scan interval and parameters
- Automatic device discovery and MAC address extraction
- Duplicate device filtering

### 2. Multi-Core Processing
- **Core 0**: MQTT communication and connection management
- **Core 1**: Bluetooth scanning and data processing

### 3. OTA Updates
- Over-the-air firmware updates via MQTT
- Topic: `traffic/bluetooth/ota`
- Supports chunked firmware transfer

### 4. Device Control
- Remote reboot via MQTT
- Status reporting
- Topic: `traffic/bluetooth/control`

## Hardware Requirements

- ESP32 development board
- WiFi connectivity
- Bluetooth Low Energy support

## Software Dependencies

```ini
lib_deps =
    knolleary/PubSubClient @ ^2.8
    ArduinoJson @ ^6.21.3
```

## Usage

### 1. Compilation and Upload
```bash
pio run --target upload
```

### 2. Monitor Output
```bash
pio device monitor
```

### 3. Expected Output
```
ESP32 BLE Point A - Starting up...
Connecting to WiFi
Connected to WiFi
Time sync done.
Device MAC: 60:07:C4:42:04:86
BLE initialized successfully
MQTT Task started on core: 0
Bluetooth Task started on core: 1
Setup complete! Both cores are now running tasks.
Starting BLE scan...
Found BLE device: 09:CE:BB:F9:3F:82
Found BLE device: 41:BB:00:EA:66:88
Found BLE device: 6B:E1:C3:26:C7:95
=== Publishing Bluetooth Data ===
JSON Payload: {"devices":["09:CE:BB:F9:3F:82","41:BB:00:EA:66:88","6B:E1:C3:26:C7:95"],"id":"60:07:C4:42:04:86","point":"A","time":1754419376}
Devices found: 3
Device MAC (ID): 60:07:C4:42:04:86
Point: A
Timestamp: 1754419376
=================================
✓ Bluetooth data published successfully
```

## MQTT Topics

| Topic | Purpose | Direction |
|-------|---------|-----------|
| `traffic/bluetooth/mac` | Bluetooth device data | ESP32 → MQTT |
| `traffic/bluetooth/ota` | OTA firmware updates | MQTT → ESP32 |
| `traffic/bluetooth/control` | Device control commands | MQTT ↔ ESP32 |

## Control Commands

### Reboot Device
```json
"reboot"
```

### Get Status
```json
"SCAN_STATUS"
```

### Status Response
```json
{
  "device_id": "esp32_ble_point_A",
  "status": "running",
  "uptime": 123456,
  "free_heap": 123456
}
```

## Troubleshooting

### 1. BLE Scanning Issues
- Ensure BLE is properly initialized
- Check for interference from other Bluetooth devices
- Verify scan parameters are appropriate for your environment

### 2. MQTT Connection Issues
- Verify WiFi credentials
- Check MQTT server connectivity
- Ensure correct MQTT credentials

### 3. JSON Publishing Issues
- Check ArduinoJson library version
- Verify JSON document size allocation
- Monitor serial output for error messages

## Testing

Run the test script to verify JSON structure:
```bash
python3 test_json_structure.py
```

## Customization

### Change Point Identifier
```cpp
const char* point = "B"; // Change from "A" to "B"
```

### Modify Scan Interval
```cpp
const unsigned long SCAN_INTERVAL = 5000; // Scan every 5 seconds
```

### Add Device Filtering
Modify the `ble_scan_callback` function to filter specific device types or MAC address ranges.

## Security Considerations

- Change default WiFi and MQTT credentials
- Use secure MQTT connections (TLS) in production
- Implement device authentication if required
- Consider MAC address privacy implications

## Performance Optimization

- Adjust scan parameters for your use case
- Monitor memory usage with large device lists
- Consider implementing device caching
- Optimize JSON payload size for network efficiency 