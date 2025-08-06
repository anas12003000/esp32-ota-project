#!/bin/bash

echo "Installing required Arduino libraries for ESP32 OTA project..."

# Create libraries directory if it doesn't exist
mkdir -p ~/Arduino/libraries

# Install PubSubClient library
echo "Installing PubSubClient library..."
cd ~/Arduino/libraries
if [ ! -d "PubSubClient" ]; then
    git clone https://github.com/knolleary/pubsubclient.git PubSubClient
    echo "PubSubClient installed successfully"
else
    echo "PubSubClient already exists"
fi

# Install ArduinoOTA (usually comes with ESP32 board package)
echo "ArduinoOTA should be included with ESP32 board package"

# Check if ESP32 board package is installed
echo "Checking ESP32 board package..."
arduino-cli core list | grep esp32 || echo "ESP32 core not found. Please install ESP32 board package first."

echo "Libraries installation complete!"
echo ""
echo "Required libraries:"
echo "- WiFi.h (built-in with ESP32)"
echo "- WiFiClient.h (built-in with ESP32)"
echo "- PubSubClient.h (installed via git)"
echo "- ArduinoOTA.h (built-in with ESP32)"
echo "- time.h (built-in)"
echo "- set (C++ standard library)"
echo "- Update.h (built-in with ESP32)"
echo "- esp_bt.h (built-in with ESP32)"
echo "- esp_bt_main.h (built-in with ESP32)"
echo "- esp_bt_device.h (built-in with ESP32)"
echo "- esp_gap_bt_api.h (built-in with ESP32)" 