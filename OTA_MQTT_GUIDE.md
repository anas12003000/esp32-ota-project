# ESP32 OTA Update Guide via MQTT

This guide explains how to perform Over-The-Air (OTA) updates using MQTT for the ESP32 BLE Scanner project.

## MQTT Topics Used

- Main topic: `traffic/bluetooth/ota`
- Control messages are sent to: `traffic/bluetooth/control`

## OTA Update Process

### 1. Prepare the New Firmware

1. Build your project in PlatformIO:

```bash
pio run
```

2. The compiled binary will be located at:

```text
.pio/build/esp32dev/firmware.bin
```

### 2. Start OTA Update

1. Send an OTA start message to initiate the update:

```json
Topic: traffic/bluetooth/ota
Message: START:${filesize}
```

Replace ${filesize} with the size of your firmware.bin in bytes.

2. Send the firmware data in chunks:
   - Split the firmware.bin into chunks
   - Publish each chunk to the OTA topic
   - The device will report progress in the serial monitor

3. Complete the update:

```json
Topic: traffic/bluetooth/ota
Message: END
```

# Python Requirements

Before using the OTA update script, install the required Python packages:

```bash
pip install paho-mqtt
```

Create a `requirements.txt` file:
```text
paho-mqtt==1.6.1
```

Or install using the requirements file:
```bash
pip install -r requirements.txt
```

### Python Script Example

```python
import paho.mqtt.client as mqtt
import time
import os

# MQTT Settings
MQTT_BROKER = "mqttiot.loranet.my"
MQTT_PORT = 1885
MQTT_USER = "iotdbuser"
MQTT_PASSWORD = "IoTdb2024"
OTA_TOPIC = "traffic/bluetooth/ota"

# Connect to MQTT
client = mqtt.Client()
client.username_pw_set(MQTT_USER, MQTT_PASSWORD)
client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Send Firmware
def send_firmware(firmware_path, chunk_size=1024):
    # Get firmware size
    firmware_size = os.path.getsize(firmware_path)
    
    # Send start message
    client.publish(OTA_TOPIC, f"START:{firmware_size}")
    time.sleep(1)
    
    # Read and send firmware
    with open(firmware_path, "rb") as file:
        while True:
            chunk = file.read(chunk_size)
            if not chunk:
                break
            client.publish(OTA_TOPIC, chunk)
            time.sleep(0.1)  # Small delay to prevent flooding
    
    # Send end message
    client.publish(OTA_TOPIC, "END")

# Usage Example
firmware_path = ".pio/build/esp32dev/firmware.bin"
send_firmware(firmware_path)
```

### Monitoring Update Progress

1. Device will print progress in serial monitor:
   ```
   [MQTT-OTA] Starting firmware update. Expected size: XXXXX bytes
   [MQTT-OTA] Progress: XX/XXXXX bytes (XX%)
   [MQTT-OTA] Update successful! Rebooting...
   ```

2. The device will automatically restart after successful update

### Error Handling

- If update fails, device will print error message and continue running old firmware
- To cancel ongoing update:
  ```json
  Topic: traffic/bluetooth/ota
  Message: CANCEL
  ```

### Security Considerations

1. MQTT broker authentication is required
2. Use secure MQTT (MQTTS) when possible
3. Verify firmware checksum before sending
4. Monitor update progress for completion

### Troubleshooting

1. **Update Fails to Start**
   - Check MQTT connection
   - Verify firmware size message format
   - Ensure sufficient free space on ESP32

2. **Update Fails During Transfer**
   - Check network stability
   - Reduce chunk size
   - Increase delay between chunks

3. **Device Crashes After Update**
   - Verify firmware compatibility
   - Check partition scheme
   - Ensure complete transfer before END message

## Additional Commands

Check device status:
```json
Topic: traffic/bluetooth/control
Message: SCAN_STATUS
```

Force device restart:
```json
Topic: traffic/bluetooth/control
Message: reboot
```

## Current Configuration

The device is configured with:
- MQTT Broker: mqttiot.loranet.my:1885
- Username: iotdbuser
- Device ID: esp32_ble_point_A
- OTA Password: MyStrongOTAPassword
